#include <SDL3/SDL.h>
#include <cassert>
#include <iostream>
#include <Eigen/Core>

#include "ds/kdtree.hpp"

using State = Eigen::Vector2f;
typedef bool (*StatePredicate)(const State&);

static const State START(0.0, 0.0);
static const State GOAL(20.0, 10.0);

int main() {
    KDTree<2, 1> kd_tree;
    kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    kd_tree.add_point(Eigen::Vector2f(1.0, 2.0));
    kd_tree.add_point(Eigen::Vector2f(3.0, 2.0));
    std::cout << kd_tree.nearest_neighbor(Eigen::Vector2f(1.7, 2.0)) << "\n";

    return 0;
}