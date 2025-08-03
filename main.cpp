#include <Eigen/Core>
#include <SDL3/SDL.h>
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <iostream>

using State = Eigen::Vector2f;
typedef bool (*StatePredicate)(const State&);

static const State START(0.0, 0.0);
static const State GOAL(20.0, 10.0);

static const State SEARCH_MIN(-50.0, -50.0);
static const State SEARCH_MAX(50.0, 50.0);

template<const size_t Dims, const size_t LeafSize>
class UniformSplittingKDTree {
private:
    using Point = Eigen::Vector<float, Dims>;

    struct Node {
        // invariant:
        // !is_leaf() <=> lower != nullptr <=> upper != nullptr <=> leaf_points == nullptr
        std::unique_ptr<Node> lower;
        std::unique_ptr<Node> upper;

        // TODO: is this a good idea? Reduces storage but pay for access?
        std::unique_ptr<std::array<int, LeafSize>> leaf_points;
        size_t num_points;

        size_t split_idx;
        float split_value;

        static Node new_leaf() {
            auto node = Node{
                .lower = nullptr,
                .upper = nullptr,
                .leaf_points = std::make_unique<std::array<int, LeafSize>>(),
                .num_points = 0,
                .split_idx = 0,
                .split_value = 0.0,
            };
            return node;
        }

        bool is_leaf() const {
            assert((lower.get() == nullptr) == (upper.get() == nullptr)
                   && (upper.get() == nullptr) == (leaf_points != nullptr));

            return leaf_points != nullptr;
        }

        bool try_add_point(size_t point_idx) {
            assert(is_leaf());

            if (full()) return false;

            (*leaf_points)[num_points] = point_idx;
            num_points++;

            return true;
        }

        bool empty() const {
            assert(is_leaf());
            return num_points == 0;
        }

        bool full() const {
            assert(is_leaf());
            return num_points == LeafSize;
        }
    };

public:
    UniformSplittingKDTree(const Point& min_bound, const Point& max_bound,
        float point_dist_tol = 1e-5)
        : min_bound_(min_bound),
          max_bound_(max_bound),
          squared_tol_(point_dist_tol * point_dist_tol) {}

    void add_point(const Point& point) {
        Point current_min = min_bound_;
        Point current_max = max_bound_;

        Node* node = &root_;
        while (!node->is_leaf()) {
            const auto& [split_idx, split_val] = find_split(current_min, current_max);

            if (point[split_idx] > split_val) {
                // can't be null, because node isn't a leaf
                node = node->upper.get();
                current_min[split_idx] = split_val;
            } else {
                node = node->lower.get();
                current_max[split_idx] = split_val;
            }
        }

        // avoid duplicate or near-duplicate points
        assert(node != nullptr);
        std::optional<size_t> closest_opt = closest_in_leaf(*node, point);
        if (closest_opt.has_value()
            && (points_[closest_opt.value()] - point).squaredNorm() < squared_tol_) {
            return;
        }

        int idx = points_.size();
        points_.push_back(point);

        // continue splitting while we can't add we're pretty likely to chop at
        // least one point off the first split if the node is full, but not guaranteed if half
        // of the cell is covered by an obstacle, for example
        while (!node->try_add_point(idx)) {
            const auto& [split_idx, split_val] = find_split(current_min, current_max);

            node->lower = std::make_unique<Node>(Node::new_leaf());
            node->upper = std::make_unique<Node>(Node::new_leaf());

            for (int i: *node->leaf_points) {
                if (points_[i][split_idx] > split_val) {
                    assert(node->upper->try_add_point(i));
                } else {
                    assert(node->lower->try_add_point(i));
                }
            }

            node->leaf_points = nullptr;

            if (point[split_idx] > split_val) {
                node = node->upper.get();
                current_min[split_idx] = split_val;
            } else {
                node = node->lower.get();
                current_max[split_idx] = split_val;
            }
        }
    }

    std::pair<Eigen::Index, float> find_split(const Point& min, const Point& max) {
        Point diff = max - min;
        Eigen::Index split_idx;
        diff.maxCoeff(&split_idx);
        Point avg = max + min / 2;
        float split_val = avg[split_idx] / 2;
        return {split_idx, split_val};
    }

    const Point& nearest_neighbor(const Point& query) const {
        assert(size() != 0);

        auto result = nn_helper(&root_, query, min_bound_, max_bound_);
        assert(result.first != nullptr);

        return *result.first;
    }

    std::pair<const Point*, float> nn_helper(const Node* node, const Point& query,
        const Point& low_bound, const Point& high_bound) const {
        assert(size() != 0);

        if (node == nullptr) {
            return {nullptr, std::numeric_limits<float>::infinity()};
        }

        if (node->is_leaf()) {
            std::optional<size_t> closest_opt = closest_in_leaf(*node, query);
            if (!closest_opt.has_value()) {
                return {nullptr, std::numeric_limits<float>::infinity()};
            }

            size_t closest_idx = closest_opt.value();
            float sq_dist = (query - points_[closest_idx]).squaredNorm();
            return {&points_[closest_idx], sq_dist};
        } else {
            Node* more_promising;
            Point more_high_bound = high_bound;
            Point more_low_bound = low_bound;

            Node* less_promising;
            Point less_high_bound = high_bound;
            Point less_low_bound = low_bound;

            if (query[node->split_idx] > node->split_value) {
                more_promising = node->upper.get();
                less_promising = node->lower.get();

                more_low_bound[node->split_idx] = node->split_value;
                less_high_bound[node->split_idx] = node->split_value;
            } else {
                more_promising = node->lower.get();
                less_promising = node->upper.get();

                more_high_bound[node->split_idx] = node->split_value;
                less_low_bound[node->split_idx] = node->split_value;
            }

            assert(more_promising != nullptr && less_promising != nullptr);

            const auto& [pt, sq_dist] = nn_helper(more_promising, query, more_low_bound,
                more_high_bound);

            float sq_border_dist = query[node->split_idx] - node->split_value;
            sq_border_dist = sq_border_dist * sq_border_dist;

            if (sq_dist < sq_border_dist) {
                return {pt, sq_dist};
            } else {
                const auto& [new_pt, new_sq_dist] = nn_helper(less_promising, query, less_low_bound,
                    less_high_bound);

                if (new_sq_dist < sq_dist) {
                    return {new_pt, new_sq_dist};
                } else {
                    return {pt, sq_dist};
                }
            }
        }
    }

    size_t size() const {
        return points_.size();
    }

private:
    std::optional<size_t> closest_in_leaf(const Node& node, const Point& query) const {
        assert(node.is_leaf());

        if (node.empty()) return {};

        return *std::min_element(node.leaf_points->begin(),
            node.leaf_points->begin() + node.num_points, [&](int a_idx, int b_idx) {
                return (points_[a_idx] - query).squaredNorm()
                       < (points_[b_idx] - query).squaredNorm();
            });
    }

    Node root_ = Node::new_leaf();
    std::vector<Point> points_;

    Point min_bound_;
    Point max_bound_;
    float squared_tol_;
};

int main() {
    UniformSplittingKDTree<2, 1> kd_tree(SEARCH_MIN, SEARCH_MAX);
    kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    kd_tree.add_point(Eigen::Vector2f(3.0, 2.0));

    return 0;
}