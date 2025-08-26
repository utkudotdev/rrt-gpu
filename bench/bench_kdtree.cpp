#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <cstddef>
#include <random>
#include <vector>

#include "ds/kdtree.hpp"
#include "util/random.hpp"

std::mt19937 gen(10);
std::uniform_real_distribution<float> dist(-1.0, 1.0);

// Insertion benchmark
template<const size_t Dims, const size_t LeafSize>
static void BM_Insertion(benchmark::State& state) {
    auto points = generate_points<Dims>(state.range(0), dist, gen);
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
    auto points = generate_points<Dims>(state.range(0), dist, gen);
    KDTree<Dims, LeafSize> tree;
    for (const auto& p: points) {
        tree.add_point(p);
    }
    auto queries = generate_points<Dims>(state.range(1), dist, gen);

    for (auto _: state) {
        for (const auto& q: queries) {
            benchmark::DoNotOptimize(tree.closest_point(q));
        }
        benchmark::ClobberMemory();
    }
    state.SetComplexityN(state.range(0));  // Complexity based on tree size
}

constexpr static size_t POINT_COUNT_MIN = 1 << 8;
constexpr static size_t POINT_COUNT_MAX = 1 << 18;
constexpr static size_t QUERY_COUNT = 1 << 8;
constexpr static int MULT = 4;

// Insertion Benchmarks
#define BENCH_INSERT(dims, leaves)                                                                 \
    BENCHMARK_TEMPLATE(BM_Insertion, dims, leaves)                                                 \
        ->RangeMultiplier(MULT)                                                                    \
        ->Range(POINT_COUNT_MIN, POINT_COUNT_MAX)                                                  \
        ->Complexity();

BENCH_INSERT(2, 1)
BENCH_INSERT(2, 4)
BENCH_INSERT(2, 16)
BENCH_INSERT(2, 64)

BENCH_INSERT(5, 1)
BENCH_INSERT(5, 4)
BENCH_INSERT(5, 16)
BENCH_INSERT(5, 64)

// Lookup Benchmarks
#define BENCH_LOOKUP(dims, leaves)                                                                 \
    BENCHMARK_TEMPLATE(BM_Lookup, dims, leaves)                                                    \
        ->RangeMultiplier(MULT)                                                                    \
        ->Ranges({{POINT_COUNT_MIN, POINT_COUNT_MAX}, {QUERY_COUNT, QUERY_COUNT}})                 \
        ->Complexity();

BENCH_LOOKUP(2, 1)
BENCH_LOOKUP(2, 4)
BENCH_LOOKUP(2, 16)
BENCH_LOOKUP(2, 64)

BENCH_LOOKUP(5, 1)
BENCH_LOOKUP(5, 4)
BENCH_LOOKUP(5, 16)
BENCH_LOOKUP(5, 64)

BENCHMARK_MAIN();
