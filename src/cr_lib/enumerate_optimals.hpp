/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2017  Florian Barth

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
#ifndef ENUMERATE_OPTIMALS_H
#define ENUMERATE_OPTIMALS_H

#include "cgaltypes.hpp"
#include "dijkstra.hpp"
#include "ilp_independent_set.hpp"
#include "restriction_policy.hpp"
#include "routeComparator.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <queue>

auto compare_prio = [](auto left, auto right) { return left.data().prio() > right.data().prio(); };

namespace c = std::chrono;

template <int Dim, class ExclusionPolicy = AllFacetsPolicy<Dim>> class EnumerateOptimals {
  public:
  using CgalDim = typename CgalTypes<Dim>::CgalDim;
  using Traits = typename CgalTypes<Dim>::Traits;
  using Vertex = typename CgalTypes<Dim>::Vertex;
  using FullCell = typename CgalTypes<Dim>::FullCell;
  using TDS = typename CgalTypes<Dim>::TDS;
  using Triangulation = typename CgalTypes<Dim>::Triangulation;
  using Facet = typename CgalTypes<Dim>::Facet;
  using VertexIter = typename CgalTypes<Dim>::VertexIter;

  using CellContainer = std::priority_queue<typename TDS::Full_cell,
      std::vector<typename TDS::Full_cell>, decltype(compare_prio)>;

  using Graph = Graph<Dim>;
  using Route = Route<Dim>;
  using Config = Config<Dim>;
  using Dijkstra = Dijkstra<Dim>;

  typedef std::vector<std::pair<size_t, size_t>> Edges;

  private:
  Graph* g;
  Triangulation tri { Dim };
  std::vector<Route> routes;
  std::vector<Config> configs;
  std::map<std::pair<size_t, size_t>, double> similarities;
  double maxOverlap;
  size_t maxRoutes;
  Dijkstra d;
  typename Dijkstra::ScalingFactor factor;
  ExclusionPolicy policy;

  double compare(size_t i, size_t j);
  Config findConfig(const typename TDS::Full_cell& f);
  void addToTriangulation();
  void includeConvexHullCells(CellContainer& cont);
  std::vector<size_t> extract_independent_set(const std::vector<size_t>& vertices, bool ilp);
  std::tuple<std::vector<size_t>, EnumerateOptimals::Edges> vertex_ids_and_edges();
  void run_base_configs(NodePos s, NodePos t);

  public:
  size_t enumeration_time;
  size_t recommendation_time;

  EnumerateOptimals(
      Graph* g, double maxOverlap, size_t maxRoutes, ExclusionPolicy excl = ExclusionPolicy());

  void find(NodePos s, NodePos t);

  size_t found_route_count() const;
  size_t vertex_count() const;

  std::tuple<std::vector<Route>, std::vector<Config>, EnumerateOptimals::Edges> recommend_routes(
      bool ilp);
};

#include "enumerate_optimals.inc"
#endif /* ENUMERATE_OPTIMALS_H */
