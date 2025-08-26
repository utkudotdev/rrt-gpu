#include <concepts>
#include <cstddef>

#include "ds/point_set.hpp"

template<typename T, const int Dims>
concept SupportsNearestNeighbor = requires(T t, T const const_t,
                                      const Eigen::Vector<float, Dims>& v) {
    { const_t.closest_point(v) } -> std::same_as<size_t>;
} and PointSet<T, Dims>;
