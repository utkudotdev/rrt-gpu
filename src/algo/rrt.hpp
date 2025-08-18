#include <Eigen/Core>
#include <cstddef>
#include <concepts>

template<typename T, const int Dims>
concept SupportsNearestPoint = requires(T t, T const const_t, T::Point v, T::PointId pid) {
    t.add_point(v);
    { const_t.closest_point(v) } -> std::same_as<typename T::PointId>;
    { const_t.get_point(pid) } -> std::same_as<typename T::Point>;
    { const_t.size() } -> std::same_as<size_t>;
};

template<typename T, const int Dims>
concept PointPredicate = std::predicate<T, const Eigen::Vector<float, Dims>&>;

template<const int Dims, SupportsNearestPoint<Dims> NPIndex, PointPredicate<Dims> Pred>
size_t rrt(NPIndex& np_index, const Pred& a) {}
