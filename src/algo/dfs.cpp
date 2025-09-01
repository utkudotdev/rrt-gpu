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

int tree_max_depth(const std::vector<std::vector<size_t>>& tree, size_t root) {
    std::vector<bool> seen(tree.size(), false);

    std::vector<std::pair<size_t, int>> stack;
    stack.push_back({root, 0});

    int max_depth = 0;
    while (!stack.empty()) {
        auto [node, depth] = stack.back();
        stack.pop_back();

        if (seen[node]) {
            return -1;
        }

        seen[node] = true;
        max_depth = std::max(max_depth, depth);

        for (auto other: tree[node]) {
            stack.push_back({other, depth + 1});
        }
    }

    return max_depth;
}
