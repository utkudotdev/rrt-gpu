#include "algo/dfs.hpp"

#include <cstddef>
#include <vector>

bool dfs_helper(const std::vector<std::vector<size_t>>& tree, size_t root, size_t goal,
    std::vector<size_t>& path) {
    path.push_back(root);
    if (root == goal) {
        return true;
    }

    for (size_t next: tree[root]) {
        bool found = dfs_helper(tree, next, goal, path);
        if (found) {
            return true;
        }
    }

    path.pop_back();
    return false;
}

std::vector<size_t> dfs(const std::vector<std::vector<size_t>>& tree, size_t root, size_t goal) {
    std::vector<size_t> path;
    dfs_helper(tree, root, goal, path);  // path should be empty by default if not found
    return path;
}
