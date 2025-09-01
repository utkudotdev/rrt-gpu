#include <doctest.h>

#include <Eigen/Core>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "algo/raytrace.hpp"

TEST_CASE("within one cell") {
    bool storage[4] = {false};
    OccupancyGridView grid(storage, 2, 2, {0, 0}, 1.0);
    grid.cell(1, 0) = true;
    grid.cell(0, 1) = true;
    grid.cell(1, 1) = true;

    CHECK(!is_segment_occupied({0.25, 0.25}, {0.75, 0.75}, grid));
}

TEST_CASE("basic multiple cells") {
    SUBCASE("unoccupied") {
        bool storage[9] = {false};
        OccupancyGridView grid(storage, 3, 3, {0, 0}, 1.0);
        grid.cell(0, 2) = true;
        grid.cell(1, 2) = true;

        CHECK(!is_segment_occupied({0.5, 0.5}, {2.7, 2.3}, grid));
    }

    SUBCASE("occupied") {
        bool storage[9] = {false};
        OccupancyGridView grid(storage, 3, 3, {0, 0}, 1.0);
        grid.cell(1, 1) = true;

        CHECK(is_segment_occupied({0.5, 0.5}, {2.7, 2.3}, grid));
    }
}

TEST_CASE("negative delta") {
    SUBCASE("negative delta x") {
        bool storage[9] = {false};
        OccupancyGridView grid(storage, 3, 3, {0, 0}, 1.0);
        grid.cell(0, 0) = true;
        grid.cell(2, 2) = true;

        CHECK(!is_segment_occupied({2.5, 0.5}, {0.5, 2.5}, grid));
    }

    SUBCASE("negative delta y") {
        bool storage[9] = {false};
        OccupancyGridView grid(storage, 3, 3, {0, 0}, 1.0);
        grid.cell(0, 0) = true;
        grid.cell(2, 2) = true;

        CHECK(!is_segment_occupied({0.5, 2.5}, {2.5, 0.5}, grid));
    }
}

TEST_CASE("shifted origin") {
    bool storage[9] = {false};
    OccupancyGridView grid(storage, 3, 3, {-1.5, -1.5}, 1.0);
    grid.cell(2, 0) = true;
    grid.cell(0, 2) = true;

    CHECK(!is_segment_occupied({-1.0, -1.0}, {1.0, 1.0}, grid));
}

TEST_CASE("special lines") {
    SUBCASE("horizontal line") {}
    SUBCASE("vertical line") {}
    SUBCASE("end at edge") {}
}

TEST_CASE("line through point corners") {
    SUBCASE("unoccupied") {
        bool storage[9] = {false};
        OccupancyGridView grid(storage, 3, 3, {-1.5, -1.5}, 1.0);

        std::vector<Eigen::Vector2f> directions{{1.5, -1.5}, {1.5, 1.5}, {-1.5, 1.5}, {-1.5, -1.5}};
        std::vector<std::pair<size_t, size_t>> occupied_cells{{1, 0}, {2, 1}, {1, 2}, {0, 1}};

        for (auto [cell_x, cell_y]: occupied_cells) {
            grid.cell(cell_x, cell_y) = true;
            for (const auto& dir: directions) {
                CHECK(!is_segment_occupied({0, 0}, dir, grid));
            }
            grid.cell(cell_x, cell_y) = false;
        }
    }

    SUBCASE("occupied") {
        bool storage[9] = {false};
        OccupancyGridView grid(storage, 3, 3, {-1.5, -1.5}, 1.0);
        grid.cell(1, 0) = true;
        grid.cell(2, 1) = true;
        grid.cell(1, 2) = true;
        grid.cell(0, 1) = true;

        std::vector<Eigen::Vector2f> directions{{1.5, -1.5}, {1.5, 1.5}, {-1.5, 1.5}, {-1.5, -1.5}};

        for (const auto& dir: directions) {
            CHECK(is_segment_occupied({0, 0}, dir, grid));
        }
    }
}
