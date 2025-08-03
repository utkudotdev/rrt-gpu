#include <doctest.h>
#include "ds/kdtree.hpp"

TEST_CASE("duplicate points are removed") {
    KDTree<2, 1> kd_tree;

    kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    kd_tree.add_point(Eigen::Vector2f(3.0, 2.0));

    CHECK(kd_tree.size() == 2);
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

        CHECK(kd_tree.nearest_neighbor(Eigen::Vector2f(-1.0, 3.5))
                .isApprox(Eigen::Vector2f(-0.9, 4.0)));
        CHECK(kd_tree.nearest_neighbor(Eigen::Vector2f(8.0, 0.1))
                .isApprox(Eigen::Vector2f(10.0, 0.2)));
    }

    SUBCASE("small leaves") {
        KDTree<2, 1> kd_tree;

        kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
        kd_tree.add_point(Eigen::Vector2f(5.0, 1.0));
        kd_tree.add_point(Eigen::Vector2f(-3.0, 8.0));
        kd_tree.add_point(Eigen::Vector2f(10.0, 0.2));
        kd_tree.add_point(Eigen::Vector2f(-0.9, 4.0));

        CHECK(kd_tree.size() == 5);

        CHECK(kd_tree.nearest_neighbor(Eigen::Vector2f(-1.0, 3.5))
                .isApprox(Eigen::Vector2f(-0.9, 4.0)));
        CHECK(kd_tree.nearest_neighbor(Eigen::Vector2f(8.0, 0.1))
                .isApprox(Eigen::Vector2f(10.0, 0.2)));
    }
}
