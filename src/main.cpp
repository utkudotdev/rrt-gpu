#include <SDL3/SDL.h>
#include <cassert>
#include <Eigen/Core>
#include <iostream>

#include "ds/kdtree.hpp"
#include "algo/rrt.hpp"

using State = Eigen::Vector2f;
typedef bool (*StatePredicate)(const State&);

static const State START(0.0, 0.0);
static const State GOAL(20.0, 10.0);

int main() {
    KDTree<2, 32> kd_tree;
    std::cout << rrt<2>(kd_tree, [](State s) { return s(0) > 0.1; }) << "\n";

    return 0;
}
