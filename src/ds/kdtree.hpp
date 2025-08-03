#include <SDL3/SDL.h>
#include <cassert>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <Eigen/Core>

template<const size_t Dims, const size_t LeafSize>
class KDTree {
private:
    static_assert(Dims > 0, "0-dimensional points are not allowed");
    static_assert(LeafSize > 0, "Leaves must carry at least one point");

    using Point = Eigen::Vector<float, Dims>;
    using LeafPoints = Eigen::Matrix<float, Dims, LeafSize>;

    struct LeafData {
        LeafPoints points;
        size_t count;
    };

    struct Node {
        // invariant:
        // !is_leaf() <=> lower != nullptr <=> upper != nullptr <=> leaf_data == nullptr
        std::unique_ptr<Node> lower;
        std::unique_ptr<Node> upper;

        std::unique_ptr<LeafData> leaf_data;

        size_t split_idx;
        float split_value;

        static Node new_leaf() {
            return Node{
                .lower = nullptr,
                .upper = nullptr,
                .leaf_data = std::make_unique<LeafData>(),
                .split_idx = 0,
                .split_value = 0.0,
            };
        }

        bool is_leaf() const {
            assert((lower.get() == nullptr) == (upper.get() == nullptr)
                   && (upper.get() == nullptr) == (leaf_data != nullptr));

            return leaf_data != nullptr;
        }

        bool try_add_point(const Point& point) {
            assert(is_leaf());

            if (full()) return false;

            leaf_data->points.col(leaf_data->count) = point;
            leaf_data->count++;

            return true;
        }

        bool empty() const {
            assert(is_leaf());
            return leaf_data->count == 0;
        }

        bool full() const {
            assert(is_leaf());
            return leaf_data->count == LeafSize;
        }
    };

public:
    KDTree(float point_dist_tol = 1e-5)
        : total_points_(0), squared_tol_(point_dist_tol * point_dist_tol) {}

    void add_point(const Point& point) {
        Node* node = &root_;
        while (!node->is_leaf()) {
            Eigen::Index split_idx = node->split_idx;
            float split_val = node->split_value;

            if (point[split_idx] > split_val) {
                // can't be null, because node isn't a leaf
                node = node->upper.get();
            } else {
                node = node->lower.get();
            }
        }

        // avoid duplicate or near-duplicate points
        assert(node != nullptr);
        auto closest_opt = closest_in_leaf(*node, point);
        if (closest_opt.has_value() && closest_opt.value().second < squared_tol_) {
            return;
        }

        // continue splitting while we can't add
        while (!node->try_add_point(point)) {
            const auto& [split_idx, split_val] = find_split(*node, point);

            node->lower = std::make_unique<Node>(Node::new_leaf());
            node->upper = std::make_unique<Node>(Node::new_leaf());

            for (auto point: node->leaf_data->points.colwise()) {
                if (point(split_idx) > split_val) {
                    assert(node->upper->try_add_point(point));
                } else {
                    assert(node->lower->try_add_point(point));
                }
            }

            node->leaf_data = nullptr;
            node->split_idx = split_idx;
            node->split_value = split_val;

            if (point[split_idx] > split_val) {
                node = node->upper.get();
            } else {
                node = node->lower.get();
            }
        }

        total_points_++;
    }

    Point nearest_neighbor(const Point& query) const {
        assert(size() != 0);

        auto result = nn_helper(&root_, query);
        return result.first;
    }

    size_t size() const {
        return total_points_;
    }

private:
    std::pair<Point, float> nn_helper(const Node* node, const Point& query) const {
        assert(size() != 0);

        if (node == nullptr) {
            return {Point(), std::numeric_limits<float>::infinity()};
        }

        if (node->is_leaf()) {
            auto closest_opt = closest_in_leaf(*node, query);
            if (!closest_opt.has_value()) {
                return {Point(), std::numeric_limits<float>::infinity()};
            }

            const auto& [point_idx, sq_dist] = closest_opt.value();
            return {node->leaf_data->points.col(point_idx), sq_dist};
        } else {
            Node* more_promising;
            Node* less_promising;

            if (query(node->split_idx) > node->split_value) {
                more_promising = node->upper.get();
                less_promising = node->lower.get();
            } else {
                more_promising = node->lower.get();
                less_promising = node->upper.get();
            }

            assert(more_promising != nullptr && less_promising != nullptr);

            const auto& [pt, sq_dist] = nn_helper(more_promising, query);

            float border_dist = query[node->split_idx] - node->split_value;
            float sq_border_dist = border_dist * border_dist;

            if (sq_dist < sq_border_dist) {
                return {pt, sq_dist};
            } else {
                const auto& [new_pt, new_sq_dist] = nn_helper(less_promising, query);

                if (new_sq_dist < sq_dist) {
                    return {new_pt, new_sq_dist};
                } else {
                    return {pt, sq_dist};
                }
            }
        }
    }

    std::optional<std::pair<Eigen::Index, float>> closest_in_leaf(const Node& node,
        const Point& query) const {
        assert(node.is_leaf());

        if (node.empty()) return {};

        auto pts_view = node.leaf_data->points.leftCols(node.leaf_data->count);

        Eigen::Index result_idx;
        float result_sq_dist =
            (pts_view.colwise() - query).colwise().squaredNorm().minCoeff(&result_idx);

        return {{result_idx, result_sq_dist}};
    }

    std::pair<Eigen::Index, float> find_split(const Node& node, const Point& to_add) {
        assert(node.is_leaf());

        auto pts_view = node.leaf_data->points.leftCols(node.leaf_data->count);
        size_t total_points = node.leaf_data->count + 1;

        Point mean = (pts_view.rowwise().sum() + to_add) / total_points;
        Point variance = ((pts_view.colwise() - mean).array().square().rowwise().sum()
                             + to_add.array().square())
                         / total_points;

        Eigen::Index split_idx;
        variance.maxCoeff(&split_idx);

        return {split_idx, mean[split_idx]};
    }

    Node root_ = Node::new_leaf();
    size_t total_points_;

    float squared_tol_;
};
