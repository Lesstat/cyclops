/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2018  Florian Barth

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "ilp_independent_set.hpp"

std::vector<size_t> find_independent_set(
    size_t nodeCount, const std::vector<std::pair<size_t, size_t>>& edges)
{

  if (edges.empty()) {
    std::vector<size_t> result;
    for (size_t i = 0; i < nodeCount; ++i) {
      result.push_back(i);
    }
    return result;
  }
  ilpSet lp(nodeCount);
  lp.addEdges(edges);
  return lp.find_set();
}

void duplicate_edges(std::vector<std::pair<size_t, size_t>>& edges)
{
  size_t initial_size = edges.size();
  for (size_t i = 0; i < initial_size; ++i) {
    auto [first, second] = edges[i];
    edges.emplace_back(second, first);
  }
}

std::vector<size_t> greedy_independent_set(
    const size_t nodeCount, std::vector<std::pair<size_t, size_t>>& edges)
{
  std::vector<size_t> result;
  std::vector<bool> picked(nodeCount, true);

  auto sort_by_degree = [&nodeCount](auto& edges) {
    std::vector<size_t> degree(nodeCount, 0);

    for (auto& [left, right] : edges) {
      degree[left]++;
      degree[right]++;
    }
    std::sort(edges.begin(), edges.end(), [&degree](const auto& left, const auto& right) {
      if (degree[left.first] == degree[right.first]) {
        return left.first < right.first;
      }
      return degree[left.first] < degree[right.first];
    });
  };

  sort_by_degree(edges);

  for (auto [first, second] : edges) {
    if (picked[first]) {
      picked[second] = false;
    }
  }

  for (size_t i = 0; i < nodeCount; ++i) {
    if (picked[i]) {
      result.push_back(i);
    }
  }

  return result;
}
