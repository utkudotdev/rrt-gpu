#include "occupancy_grid.hpp"

#include <cassert>

OccupancyGridView::OccupancyGridView(bool* storage, size_t x_cells, size_t y_cells,
    const Eigen::Vector2f& origin, float resolution)
    : storage_(storage),
      x_cells_(x_cells),
      y_cells_(y_cells),
      origin_(origin),
      resolution_(resolution) {}

bool& OccupancyGridView::cell(size_t x, size_t y) {
    assert(x < x_cells_ && y < y_cells_);
    return storage_[y * x_cells_ + x];
}

const bool& OccupancyGridView::cell(size_t x, size_t y) const {
    assert(x < x_cells_ && y < y_cells_);
    return storage_[y * x_cells_ + x];
}

std::pair<size_t, size_t> OccupancyGridView::position_to_cell(const Eigen::Vector2f& pos) const {
    assert((origin_.array() <= pos.array()).all());
    assert((pos.array() <= (origin_ + real_size()).array()).all());

    Eigen::Vector2f cell_f = (pos - origin_) / resolution_;
    return {static_cast<size_t>(cell_f.x()), static_cast<size_t>(cell_f.y())};
}

std::pair<size_t, size_t> OccupancyGridView::size() const {
    return {x_cells_, y_cells_};
}

Eigen::Vector2f OccupancyGridView::real_size() const {
    return {x_cells_ * resolution_, y_cells_ * resolution_};
}

Eigen::Vector2f OccupancyGridView::origin() const {
    return origin_;
}

float OccupancyGridView::resolution() const {
    return resolution_;
}
