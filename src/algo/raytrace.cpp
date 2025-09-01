#include "algo/raytrace.hpp"

#include <cassert>
#include <cmath>
#include <cstdlib>

#include "ds/occupancy_grid.hpp"

bool is_segment_occupied(const Eigen::Vector2f& a, const Eigen::Vector2f& b,
    const OccupancyGridView& grid) {
    auto [cell_x, cell_y] = grid.position_to_cell(a);

    // line can be reparameterized as f(t) = t * delta + a where t: [0, 1]
    float t = 0.0;
    Eigen::Vector2f delta = b - a;

    while (true) {
        if (grid.cell(cell_x, cell_y)) {
            return true;
        }

        Eigen::Vector2f current = t * delta + a;

        float next_x_intersection = (delta.x() > 0 ? cell_x + 1 : cell_x) * grid.resolution()
                                    + grid.origin().x();
        float remaining_x_t = (next_x_intersection - current.x()) / delta.x();

        float next_y_intersection = (delta.y() > 0 ? cell_y + 1 : cell_y) * grid.resolution()
                                    + grid.origin().y();
        float remaining_y_t = (next_y_intersection - current.y()) / delta.y();

        // sometims if delta is 0 it can mess up the sign
        if (std::isinf(remaining_x_t)) {
            remaining_x_t = std::abs(remaining_x_t);
        }
        if (std::isinf(remaining_y_t)) {
            remaining_y_t = std::abs(remaining_y_t);
        }

        assert(t >= 0.0);
        assert(t <= 1.0);
        assert(remaining_x_t >= 0.0);
        assert(remaining_y_t >= 0.0);
        // but not necessarily <= 1.0

        size_t x_increment = delta.x() > 0 ? 1 : -1;
        size_t y_increment = delta.y() > 0 ? 1 : -1;

        if (1.0 - t < remaining_x_t && 1.0 - t < remaining_y_t) {
            // we'll reach the end strictly before any intersection
            // means the end is in the current cell which we've already checked, so exit
            break;
        } else if (remaining_x_t == remaining_y_t) {  // extremely rare
            // in this case, we can basically choose our path
            // but we want to avoid "leaking" through so we shouldn't do +1 on each
            // if both our blocked we'll collide next iteration anyways
            if (grid.cell(cell_x + x_increment, cell_y)) {
                t += remaining_y_t;
                cell_y += y_increment;
            } else {
                t += remaining_x_t;
                cell_x += x_increment;
            }
        } else if (remaining_x_t > remaining_y_t) {
            t += remaining_y_t;
            cell_y += y_increment;
        } else {
            t += remaining_x_t;
            cell_x += x_increment;
        }
    }

    return false;
}
