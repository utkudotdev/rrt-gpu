#pragma once

#include <Eigen/Core>

#include "ds/occupancy_grid.hpp"

bool is_segment_occupied(const Eigen::Vector2f& a, const Eigen::Vector2f& b,
    const OccupancyGridView& grid);
