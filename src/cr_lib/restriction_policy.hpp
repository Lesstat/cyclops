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

#include "cgaltypes.hpp"
#include "dijkstra.hpp"

template <int Dim> struct FacetExclusionPolicy {
  using FullCell = typename CgalTypes<Dim>::TDS::Full_cell;

  bool exclude_facet(const FullCell& cell);
  bool exclude_route(const Route<Dim>& route);
  void register_route(const Route<Dim>&) {};
};

template <int Dim> struct AllFacetsPolicy : FacetExclusionPolicy<Dim> {
  using FullCell = typename CgalTypes<Dim>::TDS::Full_cell;

  bool exclude_facet(const FullCell&) { return false; };
  bool exclude_route(const Route<Dim>&) { return false; };
};

struct ImportantMetric {
  size_t metric;
  double slack;
};

template <int Dim> struct Slack : std::array<double, Dim> {
};

template <int Dim> Slack<Dim> important_metrics_to_array(std::vector<ImportantMetric> metrics)
{
  Slack<Dim> slack;
  slack.fill(std::numeric_limits<double>::max());

  for (const auto& metric : metrics) {
    slack[metric.metric] = metric.slack;
  }

  return slack;
}

template <int Dim> struct ThresholdPolicy : FacetExclusionPolicy<Dim> {
  using FullCell = typename CgalTypes<Dim>::TDS::Full_cell;
  Cost<Dim> min_cost;
  Slack<Dim> slack;

  ThresholdPolicy(Slack<Dim>& slack)
      : min_cost(std::vector<double>(Dim, std::numeric_limits<double>::max()))
      , slack(slack)
  {
  }

  bool exclude_facet(const FullCell& cell)
  {
    using Vertex_handle = typename CgalTypes<Dim>::TDS::Vertex_const_handle;

    Cost<Dim> best_cost = std::vector(Dim, std::numeric_limits<double>::max());

    for (auto v = cell.vertices_begin(); v != cell.vertices_end(); ++v) {
      auto& vertex = *v;
      if (vertex == Vertex_handle()) {
        continue;
      }
      auto& p = vertex->point();

      if (is_infinite(p))
        continue;

      int index = -1;
      for (auto cord = p.cartesian_begin(); cord != p.cartesian_end(); ++cord) {
        ++index;
        if (*cord < best_cost.values[index])
          best_cost.values[index] = *cord;
      }
    }
    for (int i = 0; i < Dim; ++i) {
      if (slack[i] < std::numeric_limits<double>::max())
        if (best_cost.values[i] > min_cost.values[i] * slack[i]) {
          return true;
        }
    }
    return false;
  };
  void register_route(const Route<Dim>& route)
  {
    const auto& route_cost = route.costs;
    for (size_t i = 0; i < Dim; ++i) {
      if (route_cost.values[i] < min_cost.values[i])
        min_cost.values[i] = route_cost.values[i];
    }
  };

  bool exclude_route(const Route<Dim>& route)
  {
    for (size_t i = 0; i < Dim; ++i) {
      if (slack[i] < std::numeric_limits<double>::max()
          && route.costs.values[i] > min_cost.values[i] * slack[i])
        return true;
    }
    return false;
  }

  bool is_infinite(const typename CgalTypes<Dim>::TDS::Vertex::Point& p)
  {
    return std::all_of(
        p.cartesian_begin(), p.cartesian_end(), [](const auto& coord) { return coord == 0; });
  }
};
