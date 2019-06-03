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
#ifndef EDGE_LOAD_H
#define EDGE_LOAD_H

#include "graph.hpp"

#include <map>
#include <vector>

template <int Dim> class EdgeLoads {
  public:
  using Route = Route<Dim>;

  EdgeLoads() = delete;
  EdgeLoads(const EdgeLoads& other) = default;
  EdgeLoads(EdgeLoads&& other) noexcept = default;
  virtual ~EdgeLoads() noexcept = default;
  EdgeLoads& operator=(const EdgeLoads& other) = default;
  EdgeLoads& operator=(EdgeLoads&& other) noexcept = default;
  EdgeLoads(const std::vector<Route>&);

  double operator[](EdgeId e);
  double max_load();
  double avg_load();

  protected:
  private:
  std::map<EdgeId, size_t> loads;
  size_t route_count;
};

template <int Dim>
EdgeLoads<Dim>::EdgeLoads(const std::vector<Route>& routes)
    : route_count(routes.size())
{
  // Optimize away special casing for the empty routes case
  if (route_count == 0) {
    route_count = 1;
  }
  for (const auto& route : routes) {
    for (const auto& edge : route.edges) {
      loads[edge]++;
    }
  }
}

template <int Dim> double EdgeLoads<Dim>::operator[](EdgeId e)
{
  return static_cast<double>(loads[e]) / route_count;
};

template <int Dim> double EdgeLoads<Dim>::max_load()
{
  auto max = *std::max_element(loads.begin(), loads.end(),
      [](const auto& a, const auto& b) { return std::get<size_t>(a) < std::get<size_t>(b); });
  return static_cast<double>(std::get<size_t>(max)) / route_count;
}

template <int Dim> double EdgeLoads<Dim>::avg_load()
{
  double sum = 0.0;
  size_t count = 0;
  for (const auto& [edge_id, load] : loads) {
    if (load == 0) {
      continue;
    }
    ++count;
    sum += this->operator[](edge_id);
  }

  if (count == 0) {
    return 0.0;
  }
  return static_cast<double>(sum) / count;
}

#endif /* EDGE_LOAD_H */
