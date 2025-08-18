#include <doctest.h>
#include "ds/kdtree.hpp"

TEST_CASE("duplicate points are removed") {
    KDTree<2, 1> kd_tree;

    auto id1 = kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    auto id2 = kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    auto id3 = kd_tree.add_point(Eigen::Vector2f(3.0, 2.0));

    CHECK(id1 == id2);
    CHECK(id2 != id3);
    CHECK(kd_tree.size() == 2);
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
}
