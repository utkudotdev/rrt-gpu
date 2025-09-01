#include <doctest.h>

#include <Eigen/Core>

#include "ds/occupancy_grid.hpp"

TEST_CASE("get cells") {
    bool storage[4] = {false};
    storage[0] = true;
    storage[3] = true;

    OccupancyGridView view(storage, 2, 2, Eigen::Vector2f::Zero(), 1.0);

    CHECK(view.cell(0, 0) == true);
    CHECK(view.cell(1, 0) == false);
    CHECK(view.cell(0, 1) == false);
    CHECK(view.cell(1, 1) == true);
}

TEST_CASE("position to cell") {
    bool storage[100];
    OccupancyGridView view(storage, 10, 10, Eigen::Vector2f(1.0, 2.0), 0.1);

    auto p1 = view.position_to_cell({1.0, 2.0});
    CHECK(p1.first == 0);
    CHECK(p1.second == 0);

    auto p2 = view.position_to_cell({1.05, 2.05});
    CHECK(p2.first == 0);
    CHECK(p2.second == 0);

    auto p3 = view.position_to_cell({1.11, 2.11});
    CHECK(p3.first == 1);
    CHECK(p3.second == 1);

    auto p4 = view.position_to_cell({1.99, 2.99});
    CHECK(p4.first == 9);
    CHECK(p4.second == 9);
}

TEST_CASE("grid properties") {
    bool storage[1];
    OccupancyGridView view(storage, 4, 5, Eigen::Vector2f(1.0, 2.0), 0.5);

    auto size = view.size();
    CHECK(size.first == 4);
    CHECK(size.second == 5);

    auto real_size = view.real_size();
    CHECK(real_size.x() == doctest::Approx(2.0));
    CHECK(real_size.y() == doctest::Approx(2.5));

    auto origin = view.origin();
    CHECK(origin.x() == doctest::Approx(1.0));
    CHECK(origin.y() == doctest::Approx(2.0));

    CHECK(view.resolution() == doctest::Approx(0.5));
}
