#include <Eigen/Core>
#include <SDL3/SDL.h>
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>
#include "Eigen/src/Core/Matrix.h"

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
        // TODO: this can actually get smaller if you use just two pointers and some kind of tag
        // could even put the tag in the pointer due to 48 bit virtual address space
        std::unique_ptr<Node> lower;
        std::unique_ptr<Node> upper;
        std::unique_ptr<std::array<int, LeafSize>> leaf_points;

        static Node new_leaf() {
            auto node = Node{
                .lower = nullptr,
                .upper = nullptr,
                .leaf_points = std::make_unique<std::array<int, LeafSize>>(),
            };
            node.leaf_points->fill(-1);
            return node;
        }

        bool is_leaf() const {
            return leaf_points != nullptr;
        }

        bool try_add_point(size_t point_idx) {
            assert(is_leaf());

            auto free_slot = std::find(leaf_points->begin(), leaf_points->end(), -1);
            if (free_slot == leaf_points->end()) return false;

            *free_slot = point_idx;

            return true;
        }

        bool empty() const {
            assert(is_leaf());
            return leaf_points->front() == -1;
        }
    };

public:
    UniformSplittingKDTree(const Point& min_bound, const Point& max_bound,
        float point_dist_tol = 1e-5)
        : min_bound_(min_bound), max_bound_(max_bound), point_dist_tol_(point_dist_tol) {}

    void add_point(const Point& point) {
        int idx = points_.size();
        points_.push_back(point);

        Point current_min = min_bound_;
        Point current_max = max_bound_;

        Node* node = &root_;
        while (!node->is_leaf()) {
            Point diff = current_max - current_min;
            Eigen::Index split_idx;
            float split_val = diff.maxCoeff(&split_idx) / 2;

            if (point[split_idx] > split_val) {
                node = node->upper.get();
                current_min[split_idx] = split_val;
            } else {
                node = node->lower.get();
                current_max[split_idx] = split_val;
            }
        }

        // avoid duplicate or near-duplicate points
        std::optional<size_t> closest_opt = closest_in_leaf(*node, point);
        if (closest_opt.has_value()) {
            size_t closest_idx = closest_opt.value();
            float squared_tol = point_dist_tol_ * point_dist_tol_;
            if ((points_[closest_idx] - point).squaredNorm() < squared_tol) {
                return;
            }
        }

        // continue splitting while we can't add we're pretty likely to chop at
        // least one point off the first split if the node is full, but not guaranteed if half
        // of the cell is covered by an obstacle, for example
        while (!node->try_add_point(idx)) {
            Point diff = current_max - current_min;
            Eigen::Index split_idx;
            float split_val = diff.maxCoeff(&split_idx) / 2;

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
        }
    }

private:
    std::optional<size_t> closest_in_leaf(const Node& node, const Point& query) {
        assert(node.is_leaf());

        if (node.empty()) {
            return {};
        }

        return *std::min_element(node.leaf_points->begin(), node.leaf_points->end(),
            [&](int a_idx, int b_idx) {
                return (points_[a_idx] - query).squaredNorm()
                       < (points_[b_idx] - query).squaredNorm();
            });
    }

    Node root_ = Node::new_leaf();
    std::vector<Point> points_;

    Point min_bound_;
    Point max_bound_;
    float point_dist_tol_;
};

int main() {
    UniformSplittingKDTree<2, 16> kd_tree(SEARCH_MIN, SEARCH_MAX);
    kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));

    return 0;
}