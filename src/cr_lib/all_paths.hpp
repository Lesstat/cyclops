/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2019  Florian Barth

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
#ifndef ALL_PATHS_H
#define ALL_PATHS_H

#include "dijkstra.hpp"
#include "graph.hpp"

#include <numeric>

template <int Dim> bool dominates(const Cost<Dim>& c1, const Cost<Dim>& c2)
{
  for (size_t i = 0; i < Dim; ++i) {
    if (c1.values[i] > c2.values[i])
      return false;
  }
  return true;
};

template <int Dim> bool is_dominated_by(const Cost<Dim>& c, const std::vector<Route<Dim>>& routes)
{
  return std::any_of(
      routes.begin(), routes.end(), [&](const auto& r) { return dominates(r.costs, c); });
}

template <int Dim, bool pareto_only = false>
std::vector<Route<Dim>> find_all_paths(Graph<Dim>& g, NodePos s, NodePos t)
{
  std::vector<Route<Dim>> result;
  std::vector<EdgeId> stack;
  std::vector<EdgeId> cur_route;
  std::vector<size_t> removal_stack;
  Cost<Dim> cur_cost;

  if constexpr (pareto_only) {
    auto d = g.createDijkstra();
    Config<Dim> conf = std::vector<double>(Dim, 0.0);
    conf.values[0] = 1.0;
    auto r = d.findBestRoute(s, t, conf);
    if (r) {
      pareto_push(result, *r);
    }
  }
  std::cout << "\n";
  for (const auto e : g.getOutgoingEdgesOf(s)) {
    stack.push_back(e.id);
  }

  while (!stack.empty()) {
    auto cur_edge = stack.back();
    stack.pop_back();
    auto next_node = Edge<Dim>::destPos(cur_edge);

    while (!removal_stack.empty() && stack.size() < removal_stack.back()) {
      cur_cost = cur_cost - Edge<Dim>::getCost(cur_route.back());
      cur_route.pop_back();
      removal_stack.pop_back();
    }
    if (Edge<Dim>::getEdgeA(cur_edge)) {
      continue;
    }

    if (std::any_of(cur_route.begin(), cur_route.end(),
            [&next_node](const EdgeId& e) { return Edge<Dim>::sourcePos(e) == next_node; })) {
      continue;
    }
    if constexpr (pareto_only) {
      if (is_dominated_by(cur_cost, result) && stack.size() % 150 == 0) {
        continue;
      }
    }

    removal_stack.push_back(stack.size());

    cur_route.push_back(cur_edge);
    cur_cost = cur_cost + Edge<Dim>::getCost(cur_edge);
    if (next_node == t) {

      std::deque<EdgeId> edges(cur_route.begin(), cur_route.end());
      Route<Dim> r;
      r.costs = cur_cost;
      r.edges = edges;
      if constexpr (pareto_only) {
        pareto_push(result, r);
      } else {
        result.push_back(r);
      }
      continue;
    }

    for (const auto e : g.getOutgoingEdgesOf(next_node)) {
      stack.push_back(e.id);
    }
  }

  std::cout << "\n";
  return result;
}

template <int Dim> bool pareto_push(std::vector<Route<Dim>>& routes, const Route<Dim>& route)
{
  if (is_dominated_by(route.costs, routes)) {
    return false;
  }

  auto pos = std::partition(routes.begin(), routes.end(),
      [&](const auto& r) { return !dominates(route.costs, r.costs); });
  routes.erase(pos, routes.end());
  routes.push_back(route);
  return true;
}

template <int Dim> std::vector<Route<Dim>> pareto_only(const std::vector<Route<Dim>>& routes)
{
  std::vector<Route<Dim>> results;

  for (const auto& route : routes) {
    pareto_push(results, route);
  }

  return results;
}

#endif /* ALL_PATHS_H */
