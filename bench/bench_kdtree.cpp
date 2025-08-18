#include <benchmark/benchmark.h>
#include "ds/kdtree.hpp"
#include <Eigen/Core>
#include <cstddef>
#include <random>
#include <vector>

// Generate uniformly distributed random points
template<const size_t Dims>
std::vector<Eigen::Vector<float, Dims>> generate_points(size_t count) {
    std::vector<Eigen::Vector<float, Dims>> points;
    points.reserve(count);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0, 1.0);

    for (size_t i = 0; i < count; ++i) {
        Eigen::Vector<float, Dims> p;
        for (size_t j = 0; j < Dims; ++j) {
            p[j] = dist(gen);
        }
        points.push_back(p);
    }
    return points;
}

// Insertion benchmark
template<const size_t Dims, const size_t LeafSize>
static void BM_Insertion(benchmark::State& state) {
    auto points = generate_points<Dims>(state.range(0));
    for (auto _: state) {
        KDTree<Dims, LeafSize> tree;
        for (const auto& p: points) {
            tree.add_point(p);
        }
    }
    state.SetComplexityN(state.range(0));
}

// Lookup benchmark
template<const size_t Dims, const size_t LeafSize>
static void BM_Lookup(benchmark::State& state) {
    auto points = generate_points<Dims>(state.range(0));
    KDTree<Dims, LeafSize> tree;
    for (const auto& p: points) {
        tree.add_point(p);
    }
    auto queries = generate_points<Dims>(state.range(1));

    size_t i = 0;
    for (auto _: state) {
        // Use different query each time
        benchmark::DoNotOptimize(tree.closest_point(queries[i]));
        i = (i + 1) % queries.size();
    }
    state.SetComplexityN(state.range(0));  // Complexity based on tree size
}

// Register benchmarks for various configurations
// Dims: 2, 3, 4
// LeafSize: 1, 4, 16, 64
// PointCount: 1000, 10000, 100000
// QueryCount: 1000 (fixed for lookup)

// Insertion Benchmarks
BENCHMARK_TEMPLATE(BM_Insertion, 2, 1)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();
BENCHMARK_TEMPLATE(BM_Insertion, 2, 4)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();
BENCHMARK_TEMPLATE(BM_Insertion, 2, 16)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();
BENCHMARK_TEMPLATE(BM_Insertion, 2, 64)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();

BENCHMARK_TEMPLATE(BM_Insertion, 3, 1)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();
BENCHMARK_TEMPLATE(BM_Insertion, 3, 4)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();
BENCHMARK_TEMPLATE(BM_Insertion, 3, 16)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();
BENCHMARK_TEMPLATE(BM_Insertion, 3, 64)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();

BENCHMARK_TEMPLATE(BM_Insertion, 4, 1)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();
BENCHMARK_TEMPLATE(BM_Insertion, 4, 4)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();
BENCHMARK_TEMPLATE(BM_Insertion, 4, 16)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();
BENCHMARK_TEMPLATE(BM_Insertion, 4, 64)->RangeMultiplier(10)->Range(1000, 100000)->Complexity();

// Lookup Benchmarks
// Args: {PointCount, QueryCount}
BENCHMARK_TEMPLATE(BM_Lookup, 2, 1)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();
BENCHMARK_TEMPLATE(BM_Lookup, 2, 4)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();
BENCHMARK_TEMPLATE(BM_Lookup, 2, 16)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();
BENCHMARK_TEMPLATE(BM_Lookup, 2, 64)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();

BENCHMARK_TEMPLATE(BM_Lookup, 3, 1)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();
BENCHMARK_TEMPLATE(BM_Lookup, 3, 4)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();
BENCHMARK_TEMPLATE(BM_Lookup, 3, 16)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();
BENCHMARK_TEMPLATE(BM_Lookup, 3, 64)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();

BENCHMARK_TEMPLATE(BM_Lookup, 4, 1)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();
BENCHMARK_TEMPLATE(BM_Lookup, 4, 4)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();
BENCHMARK_TEMPLATE(BM_Lookup, 4, 16)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();
BENCHMARK_TEMPLATE(BM_Lookup, 4, 64)
    ->Args({1000, 1000})
    ->Args({10000, 1000})
    ->Args({100000, 1000})
    ->Complexity();

BENCHMARK_MAIN();
