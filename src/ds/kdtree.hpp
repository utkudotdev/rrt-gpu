#pragma once

#include <Eigen/Core>
#include <array>
#include <cassert>
#include <cstddef>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "ds/supports_nn.hpp"

template<const size_t Dims, const size_t LeafSize>
class KDTree {
    using Point = Eigen::Vector<float, Dims>;
    using LeafPoints = Eigen::Matrix<float, Dims, LeafSize>;
    using NodeId = size_t;

    struct SplitNode {
        NodeId lower;
        NodeId upper;

        Eigen::Index split_idx;
        float split_value;
    };

    struct LeafNode {
        LeafPoints points;
        std::array<size_t, LeafSize> point_ids;
        size_t count;

        bool try_add_point(const Point& point, size_t id) {
            if (full()) return false;
            points.col(count) = point;
            point_ids[count] = id;
            count++;
            return true;
        }

        bool empty() const {
            return count == 0;
        }

        bool full() const {
            return count == LeafSize;
        }

        std::optional<std::pair<Eigen::Index, float>> closest_point(const Point& query) const {
            if (empty()) return {};

            Eigen::Index result_idx;
            float result_sq_dist = (points.leftCols(count).colwise() - query)
                                       .colwise()
                                       .squaredNorm()
                                       .minCoeff(&result_idx);

            return {{result_idx, result_sq_dist}};
        }

        std::pair<Eigen::Index, float> find_split(const Point& to_add) const {
            auto pts_view = points.leftCols(count);
            size_t total_points = count + 1;

            Point mean = (pts_view.rowwise().sum() + to_add) / total_points;
            Point leaf_sq_sum = (pts_view.colwise() - mean).array().square().rowwise().sum();
            Point to_add_sq = (to_add - mean).array().square();
            Point variance = (leaf_sq_sum + to_add_sq) / total_points;

            Eigen::Index split_idx;
            variance.maxCoeff(&split_idx);

            return {split_idx, mean[split_idx]};
        }
    };

    using Node = std::variant<SplitNode, LeafNode>;

    constexpr static float LEAF_SQUARED_TOL = 1e-7;

public:
    KDTree(): nodes_({LeafNode{}}) {}

    bool add_point(const Point& point) {
        size_t new_id = point_id_to_tree_loc_.size();
        NodeId node_id = 0;

        while (std::holds_alternative<SplitNode>(nodes_[node_id])) {
            // Traverse split node
            const SplitNode& split = std::get<SplitNode>(nodes_[node_id]);
            if (point(split.split_idx) > split.split_value) {
                node_id = split.upper;
            } else {
                node_id = split.lower;
            }
        }

        LeafNode& leaf = std::get<LeafNode>(nodes_[node_id]);
        auto closest_opt = leaf.closest_point(point);
        if (closest_opt.has_value() && closest_opt.value().second < LEAF_SQUARED_TOL) {
            return false;
        }

        if (leaf.try_add_point(point, new_id)) {
            // Update location mapping: node_id * LeafSize + index in leaf
            point_id_to_tree_loc_.push_back(node_id * LeafSize + (leaf.count - 1));
            return true;
        }

        // Leaf is full, need to split
        const auto [split_idx, split_value] = leaf.find_split(point);
        NodeId lower_id = add_leaf();
        NodeId upper_id = add_leaf();

        // other reference may now be invalidated
        const LeafNode& old_leaf = std::get<LeafNode>(nodes_[node_id]);

        // Distribute points from original leaf to new leaves
        for (size_t i = 0; i < old_leaf.count; i++) {
            const Point& p = old_leaf.points.col(i);
            size_t pid = old_leaf.point_ids[i];
            NodeId target_id = (p(split_idx) > split_value) ? upper_id : lower_id;
            LeafNode& target_leaf = std::get<LeafNode>(nodes_[target_id]);

            // Should always succeed since leaves are empty
            assert(target_leaf.try_add_point(p, pid));

            // Update location mapping
            point_id_to_tree_loc_[pid] = target_id * LeafSize + (target_leaf.count - 1);
        }

        // Now handle the new point that caused the split
        NodeId new_target_id = (point(split_idx) > split_value) ? upper_id : lower_id;
        LeafNode& new_target_leaf = std::get<LeafNode>(nodes_[new_target_id]);
        assert(new_target_leaf.try_add_point(point, new_id));
        point_id_to_tree_loc_.push_back(new_target_id * LeafSize + (new_target_leaf.count - 1));

        // Convert current node to split node
        nodes_[node_id].template emplace<SplitNode>(lower_id, upper_id, split_idx, split_value);

        return true;
    }

    size_t closest_point(const Point& query) const {
        return nn_helper(0, query).first;
    }

    Point operator[](size_t i) const {
        size_t tree_loc = point_id_to_tree_loc_[i];
        const LeafNode& leaf = std::get<LeafNode>(nodes_[tree_loc / LeafSize]);
        return leaf.points.col(tree_loc % LeafSize);
    }

    size_t size() const {
        return point_id_to_tree_loc_.size();
    }

    void clear() {
        nodes_.clear();
        point_id_to_tree_loc_.clear();
    }

private:
    std::pair<size_t, float> nn_helper(NodeId node, const Point& query) const {
        const Node& current_node = nodes_[node];
        if (std::holds_alternative<LeafNode>(current_node)) {
            const LeafNode& leaf = std::get<LeafNode>(current_node);
            if (leaf.empty()) {
                return {0, std::numeric_limits<float>::infinity()};
            }
            auto opt = leaf.closest_point(query);
            if (!opt) {
                return {0, std::numeric_limits<float>::infinity()};
            }
            auto [idx, sq_dist] = *opt;
            return {leaf.point_ids[idx], sq_dist};
        } else {
            const SplitNode& split = std::get<SplitNode>(current_node);
            float query_val = query[split.split_idx];
            NodeId near_child, far_child;
            if (query_val <= split.split_value) {
                near_child = split.lower;
                far_child = split.upper;
            } else {
                near_child = split.upper;
                far_child = split.lower;
            }

            auto [best_id, best_dist_sq] = nn_helper(near_child, query);

            // Check if we need to search the far child
            float dist_to_plane = query_val - split.split_value;
            float dist_sq_to_plane = dist_to_plane * dist_to_plane;
            if (dist_sq_to_plane < best_dist_sq) {
                auto [candidate_id, candidate_dist_sq] = nn_helper(far_child, query);
                if (candidate_dist_sq < best_dist_sq) {
                    best_id = candidate_id;
                    best_dist_sq = candidate_dist_sq;
                }
            }

            return {best_id, best_dist_sq};
        }
    }

    NodeId add_leaf() {
        NodeId id = nodes_.size();
        nodes_.emplace_back(std::in_place_type<LeafNode>);
        LeafNode& leaf = std::get<LeafNode>(nodes_.back());
        leaf.count = 0;
        return id;
    }

    std::vector<Node> nodes_;
    std::vector<size_t> point_id_to_tree_loc_;

    static_assert(SupportsNearestNeighbor<KDTree<Dims, LeafSize>, Dims>,
        "KDTree should support nearest neighbor operations.");
};
