#include <Eigen/Core>
#include <concepts>
#include <cstddef>
#include <vector>

#include "algo/dfs.hpp"
#include "ds/supports_nn.hpp"
#include "util/random.hpp"

template<typename T, const int Dims>
concept PointPredicate = std::predicate<T, const Eigen::Vector<float, Dims>&>;

template<const int Dims, SupportsNearestNeighbor<Dims> NNIndex, PointPredicate<Dims> Pred,
    typename Gen>
std::vector<size_t> rrt(const Eigen::Vector<float, Dims>& start,
    const Eigen::Vector<float, Dims>& goal, size_t num_points, float move_dist,
    const Eigen::Vector<float, Dims>& min_bound, const Eigen::Vector<float, Dims>& max_bound,
    float sq_dist_tol, NNIndex& nn_index, const Pred& is_point_free, Gen generator) {
    using Point = Eigen::Vector<float, Dims>;

    size_t start_id = nn_index.add_point(start);

    std::vector<std::vector<size_t>> tree(num_points + 2, std::vector<size_t>());

    bool found = false;
    size_t end_id = 0;

    size_t i = 0;
    while (i < num_points) {
        Point conf = generate_point_with_bounds<Dims>(min_bound, max_bound, generator);

        auto nearest_id = nn_index.closest_point(conf);
        Point nearest = nn_index[nearest_id];

        Point direction = (conf - nearest).normalized();
        Point in_between = nearest + direction * move_dist;
        if (!is_point_free(in_between)) {
            continue;
        }

        auto new_id = nn_index.add_point(in_between);

        if (nearest_id != new_id) {
            tree[nearest_id].push_back(new_id);
        }

        if ((in_between - goal).squaredNorm() < sq_dist_tol) {
            found = true;
            end_id = nn_index.add_point(goal);
            if (new_id != end_id) {
                tree[new_id].push_back(end_id);
            }

            break;
        }

        i++;
    }

    if (!found) return {};

    return dfs(tree, start_id, end_id);
}
