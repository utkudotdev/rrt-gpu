#include <Eigen/Core>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <optional>
#include <vector>

#include "algo/dfs.hpp"
#include "ds/supports_nn.hpp"
#include "util/random.hpp"

template<typename T, const int Dims>
concept EdgePredicate =
    std::predicate<T, const Eigen::Vector<float, Dims>&, const Eigen::Vector<float, Dims>&>;

struct RRTResult {
    std::vector<std::vector<size_t>> tree;
    size_t start_idx;
    std::optional<std::vector<size_t>> path;
};

template<const int Dims, SupportsNearestNeighbor<Dims> NNIndex, EdgePredicate<Dims> Pred,
    typename Gen>
RRTResult rrt(const Eigen::Vector<float, Dims>& start, const Eigen::Vector<float, Dims>& goal,
    size_t num_points, float move_dist, const Eigen::Vector<float, Dims>& min_bound,
    const Eigen::Vector<float, Dims>& max_bound, float sq_dist_tol, NNIndex& nn_index,
    const Pred& is_edge_free, Gen generator) {
    using Point = Eigen::Vector<float, Dims>;

    assert(nn_index.size() == 0);
    size_t start_idx = 0;
    assert(nn_index.add_point(start));

    std::vector<std::vector<size_t>> tree(num_points + 1, std::vector<size_t>());

    bool found = false;
    size_t end_idx = 0;

    while (nn_index.size() < num_points) {
        Point conf = generate_point_with_bounds<Dims>(min_bound, max_bound, generator);

        auto nearest_idx = nn_index.closest_point(conf);
        Point nearest = nn_index[nearest_idx];

        Point direction = (conf - nearest).normalized();
        Point in_between = nearest + direction * move_dist;
        if ((in_between.array() >= max_bound.array()).any()
            || (in_between.array() < min_bound.array()).any()) {
            continue;
        }

        if (!is_edge_free(nearest, in_between)) {
            continue;
        }

        if (!nn_index.add_point(in_between)) {
            // failed to add point
            continue;
        }
        size_t new_idx = nn_index.size() - 1;

        tree[nearest_idx].push_back(new_idx);

        if ((in_between - goal).squaredNorm() < sq_dist_tol) {
            found = true;
            end_idx = new_idx;
            break;
        }
    }

    if (!found) {
        return RRTResult{.tree = tree, .start_idx = start_idx, .path = {}};
    }

    auto path = dfs(tree, start_idx, end_idx);
    assert(!path.empty());

    return RRTResult{.tree = tree, .start_idx = start_idx, .path = path};
}

// seed: 1481585100 grid assert failed
