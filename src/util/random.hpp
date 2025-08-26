#include <Eigen/Core>
#include <cstddef>
#include <random>

template<const int Dims, typename Dist, typename Gen>
Eigen::Vector<float, Dims> generate_point(Dist distribution, Gen& generator) {
    static_assert(Dims > 0, "dimension must be at least 1");
    Eigen::Vector<float, Dims> point;
    for (size_t j = 0; j < Dims; ++j) {
        point(j) = distribution(generator);
    }
    return point;
}

template<const int Dims, typename Dist, typename Gen>
std::vector<Eigen::Vector<float, Dims>> generate_points(size_t count, Dist distribution,
    Gen& generator) {
    std::vector<Eigen::Vector<float, Dims>> points;
    points.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        points.push_back(generate_point<Dims>(distribution, generator));
    }
    return points;
}

template<const int Dims, typename Gen>
Eigen::Vector<float, Dims> generate_point_with_bounds(const Eigen::Vector<float, Dims>& min_values,
    const Eigen::Vector<float, Dims>& max_values, Gen& generator) {
    static_assert(Dims > 0, "dimension must be at least 1");
    Eigen::Vector<float, Dims> point;

    for (size_t j = 0; j < Dims; ++j) {
        std::uniform_real_distribution<float> distribution(min_values(j), max_values(j));
        point(j) = distribution(generator);
    }
    return point;
}

template<const int Dims, typename Gen>
std::vector<Eigen::Vector<float, Dims>> generate_points_with_bounds(size_t count,
    const Eigen::Vector<float, Dims>& min_values, const Eigen::Vector<float, Dims>& max_values,
    Gen& generator) {
    std::vector<Eigen::Vector<float, Dims>> points;
    points.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        points.push_back(generate_point_with_bounds<Dims>(min_values, max_values, generator));
    }
    return points;
}
