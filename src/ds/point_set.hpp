#pragma once

#include <Eigen/Core>
#include <concepts>
#include <cstddef>

template<typename T, const int Dims>
concept PointSet = requires(T t, T const const_t, const Eigen::Vector<float, Dims>& v, size_t i) {
    { t.add_point(v) } -> std::same_as<bool>;
    { const_t[i] } -> std::same_as<Eigen::Vector<float, Dims>>;
    { const_t.size() } -> std::same_as<size_t>;
};
