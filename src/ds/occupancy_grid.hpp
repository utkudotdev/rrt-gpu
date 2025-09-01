#pragma once

#include <Eigen/Core>
#include <cassert>
#include <cstddef>
#include <utility>

class OccupancyGridView {
public:
    /**
     * @brief Create a new occupancy grid view from existing data.
     *
     * @param storage The data to view as an occupancy grid. Note that this class is non-owning--it
     * will not manage this memory for you. This is beneficial to maintain compatibility with CUDA,
     * which doesn't always follow the C++ memory model
     * (https://docs.nvidia.com/cuda/cuda-c-programming-guide/#global-function-argument-processing).
     * `storage` will be interpreted in x-first order, i.e. such that storage[0] is cell <0, 0> and
     * storage[1] is cell <1, 0>.
     * @param x_cells The number of cells in the x direction.
     * @param y_cells The number of cells in the y direction.
     * @param origin The origin of the grid. The outer corner of cell <0, 0> corresponds to this
     * point.
     * @param resolution The resolution of the grid. Each grid cell will be a `resolution *
     * resolution` area.
     */
    OccupancyGridView(bool* storage, size_t x_cells, size_t y_cells, const Eigen::Vector2f& origin,
        float resolution);

    bool& cell(size_t x, size_t y);
    const bool& cell(size_t x, size_t y) const;
    std::pair<size_t, size_t> position_to_cell(const Eigen::Vector2f& pos) const;
    std::pair<size_t, size_t> size() const;
    Eigen::Vector2f real_size() const;
    Eigen::Vector2f origin() const;
    float resolution() const;

private:
    // used in x-first order (<0, 0> and <1, 0> are adjacent)
    bool* storage_;
    size_t x_cells_;
    size_t y_cells_;

    Eigen::Vector2f origin_;
    float resolution_;
};
