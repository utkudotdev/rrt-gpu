#pragma once

#include <cstddef>
#include <vector>

std::vector<size_t> dfs(const std::vector<std::vector<size_t>>& tree, size_t root, size_t goal);

int tree_max_depth(const std::vector<std::vector<size_t>>& tree, size_t root);
