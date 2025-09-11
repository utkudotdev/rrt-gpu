#include <doctest.h>

#include <cstddef>
#include <random>
#include <vector>

#include "ds/kdtree.hpp"
#include "util/random.hpp"

TEST_CASE("far points get different ids") {
    KDTree<2, 1> kd_tree;

    // second point is not required to be added correctly
    bool added1 = kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    bool added3 = kd_tree.add_point(Eigen::Vector2f(3.0, 2.0));

    CHECK(added1);
    CHECK(added3);
}

std::mt19937 gen(10);
std::uniform_real_distribution<float> dist(-1.0, 1.0);

template<const size_t LeafSize>
void test_random_inserts_and_queries(size_t count_points, size_t count_queries) {
    auto points = generate_points<2>(count_points, dist, gen);
    auto queries = generate_points<2>(count_queries, dist, gen);

    KDTree<2, LeafSize> kd_tree;
    std::vector<size_t> ids;

    for (size_t i = 0; i < points.size(); i++) {
        assert(kd_tree.add_point(points[i]));
        ids.push_back(kd_tree.size() - 1);
    }

    for (const auto& q: queries) {
        float min_dist = std::numeric_limits<float>::max();
        size_t min_id = 0;

        for (size_t id = 0; id < kd_tree.size(); id++) {
            const auto& p = kd_tree[id];
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
        kd_tree.add_point(Eigen::Vector2f(10.0, 0.2));
        kd_tree.add_point(Eigen::Vector2f(-0.9, 4.0));

        CHECK(kd_tree.size() == 5);

        CHECK(kd_tree.closest_point(Eigen::Vector2f(8.0, 0.1)) == 3);
        CHECK(kd_tree.closest_point(Eigen::Vector2f(-1.0, 3.5)) == 4);
    }

    SUBCASE("small leaves") {
        KDTree<2, 1> kd_tree;

        kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
        kd_tree.add_point(Eigen::Vector2f(5.0, 1.0));
        kd_tree.add_point(Eigen::Vector2f(-3.0, 8.0));
        kd_tree.add_point(Eigen::Vector2f(10.0, 0.2));
        kd_tree.add_point(Eigen::Vector2f(-0.9, 4.0));

        CHECK(kd_tree.size() == 5);

        CHECK(kd_tree.closest_point(Eigen::Vector2f(8.0, 0.1)) == 3);
        CHECK(kd_tree.closest_point(Eigen::Vector2f(-1.0, 3.5)) == 4);
    }

    SUBCASE("small leaf close points") {
        KDTree<2, 1> kd_tree;

        kd_tree.add_point(Eigen::Vector2f(5.0, 1.0));
        kd_tree.add_point(Eigen::Vector2f(5.0, 1.01));

        CHECK(kd_tree.closest_point(Eigen::Vector2f(5.0, 0.999)) == 0);
        CHECK(kd_tree.closest_point(Eigen::Vector2f(5.0, 1.02)) == 1);
    }

    SUBCASE("random inserts [LeafPoints=1]") {
        test_random_inserts_and_queries<1>(1000, 100);
    }

    SUBCASE("random inserts [LeafPoints=32]") {
        test_random_inserts_and_queries<32>(1000, 100);
    }
}
