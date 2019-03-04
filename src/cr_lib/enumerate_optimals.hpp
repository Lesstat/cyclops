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

#include "dijkstra.hpp"
#include "ilp_independent_set.hpp"
#include "routeComparator.hpp"

#include <CGAL/Epick_d.h>
#include <CGAL/Triangulation.h>
#include <CGAL/Triangulation_ds_full_cell.h>
#include <CGAL/point_generators_d.h>
#include <CGAL/random_selection.h>
#include <Eigen/Dense>

#include <chrono>
#include <queue>

class FullCellId {
  private:
  static int currentId;
  static std::vector<int> alive_;
  static std::vector<bool> checked_;
  static std::vector<double> prio_;

  int id_;

  public:
  FullCellId();

  FullCellId(const FullCellId& other);

  FullCellId& operator=(const FullCellId& rhs);

  FullCellId& operator=(const FullCellId&& rhs);

  FullCellId(FullCellId&& other);

  ~FullCellId();

  bool alive() const;
  bool checked() const;
  void checked(bool check);
  size_t id() const;
  double prio() const;
  void prio(double p) const;

  bool operator==(const FullCellId& other) const;
};

struct VertexData {
  size_t id;
};

auto compare_prio = [](auto left, auto right) { return left.data().prio() > right.data().prio(); };

namespace c = std::chrono;

struct ImportantMetric {
  size_t metric;
  double slack;
};

template <int Dim> class EnumerateOptimals {
  public:
  using CgalDim = CGAL::Dimension_tag<Dim>;
  using Traits = CGAL::Epick_d<CgalDim>;
  using Vertex = CGAL::Triangulation_vertex<Traits, VertexData>;
  using FullCell = CGAL::Triangulation_full_cell<Traits, FullCellId>;
  using TDS = CGAL::Triangulation_data_structure<CgalDim, Vertex, FullCell>;
  using Triangulation = CGAL::Triangulation<Traits, TDS>;
  using Facet = typename Triangulation::Facet;
  using VertexIter = typename TDS::Vertex_iterator;
  using CellContainer = std::priority_queue<typename TDS::Full_cell,
      std::vector<typename TDS::Full_cell>, decltype(compare_prio)>;

  using Important = std::array<bool, Dim>;
  using Slack = std::array<double, Dim>;

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
  Important important;
  Slack slack;

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

  EnumerateOptimals(Graph* g, double maxOverlap, size_t maxRoutes);
  EnumerateOptimals(
      Graph* g, double maxOverlap, size_t maxRoutes, Important important, Slack slack);

  void find(NodePos s, NodePos t);

  size_t found_route_count() const;
  size_t vertex_count() const;

  std::tuple<std::vector<Route>, std::vector<Config>, EnumerateOptimals::Edges> recommend_routes(
      bool ilp);

  static std::pair<Important, Slack> important_metrics_to_arrays(
      std::vector<ImportantMetric> metrics);
};

#include "enumerate_optimals.inc"
#endif /* ENUMERATE_OPTIMALS_H */
