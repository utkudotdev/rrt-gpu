#include <doctest.h>
#include <cstddef>
#include <random>
#include <vector>
#include <limits>
#include "ds/kdtree.hpp"

TEST_CASE("far points get different ids") {
    KDTree<2, 1> kd_tree;

    // close points may get the same id or may not, don't really care
    // but we do care that points far enough apart are distinct
    kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    auto id2 = kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    auto id3 = kd_tree.add_point(Eigen::Vector2f(3.0, 2.0));

    CHECK(id2 != id3);
}

template<const size_t Dims>
std::vector<Eigen::Vector<float, Dims>> generate_points(size_t count, int seed) {
    std::vector<Eigen::Vector<float, Dims>> points;
    points.reserve(count);
    std::mt19937 gen(seed);
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

template<const size_t LeafSize>
void test_random_inserts_and_queries(size_t count_points, size_t count_queries, int seed_points,
    int seed_queries) {
    auto points = generate_points<2>(count_points, seed_points);
    auto queries = generate_points<2>(count_queries, seed_queries);

    KDTree<2, LeafSize> kd_tree;
    std::vector<size_t> ids;

    for (size_t i = 0; i < points.size(); i++) {
        size_t id = kd_tree.add_point(points[i]);
        ids.push_back(id);
    }

    for (const auto& q: queries) {
        float min_dist = std::numeric_limits<float>::max();
        size_t min_id = 0;

        for (size_t id = 0; id < kd_tree.size(); id++) {
            const auto& p = kd_tree.get_point(id);
            float d = (q - p).squaredNorm();
            if (d < min_dist) {
                min_dist = d;
                min_id = id;
            }
        }

        size_t found_id = kd_tree.closest_point(q);
        CHECK(min_id == found_id);
    }
}

TEST_CASE("closest points can be looked up") {
    SUBCASE("larger leaves") {
        KDTree<2, 4> kd_tree;

        kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
        kd_tree.add_point(Eigen::Vector2f(5.0, 1.0));
        kd_tree.add_point(Eigen::Vector2f(-3.0, 8.0));
        auto id4 = kd_tree.add_point(Eigen::Vector2f(10.0, 0.2));
        auto id5 = kd_tree.add_point(Eigen::Vector2f(-0.9, 4.0));

        CHECK(kd_tree.size() == 5);

        CHECK(kd_tree.closest_point(Eigen::Vector2f(8.0, 0.1)) == id4);
        CHECK(kd_tree.closest_point(Eigen::Vector2f(-1.0, 3.5)) == id5);
    }

    SUBCASE("small leaves") {
        KDTree<2, 1> kd_tree;

        kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
        kd_tree.add_point(Eigen::Vector2f(5.0, 1.0));
        kd_tree.add_point(Eigen::Vector2f(-3.0, 8.0));
        auto id4 = kd_tree.add_point(Eigen::Vector2f(10.0, 0.2));
        auto id5 = kd_tree.add_point(Eigen::Vector2f(-0.9, 4.0));

        CHECK(kd_tree.size() == 5);

        CHECK(kd_tree.closest_point(Eigen::Vector2f(8.0, 0.1)) == id4);
        CHECK(kd_tree.closest_point(Eigen::Vector2f(-1.0, 3.5)) == id5);
    }

    SUBCASE("small leaf close points") {
        KDTree<2, 1> kd_tree;

        auto id1 = kd_tree.add_point(Eigen::Vector2f(5.0, 1.0));
        auto id2 = kd_tree.add_point(Eigen::Vector2f(5.0, 1.01));

        CHECK(kd_tree.closest_point(Eigen::Vector2f(5.0, 0.999)) == id1);
        CHECK(kd_tree.closest_point(Eigen::Vector2f(5.0, 1.02)) == id2);
    }

    SUBCASE("random inserts [LeafPoints=1]") {
        test_random_inserts_and_queries<1>(1000, 100, 10, 20);
    }

    SUBCASE("random inserts [LeafPoints=32]") {
        test_random_inserts_and_queries<32>(1000, 100, 10, 20);
    }
}
