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

typedef CGAL::Dimension_tag<DIMENSION> Dim;
typedef CGAL::Epick_d<Dim> Traits;
typedef CGAL::Triangulation_vertex<Traits, VertexData> Vertex;
typedef CGAL::Triangulation_full_cell<Traits, FullCellId> FullCell;
typedef CGAL::Triangulation_data_structure<Dim, Vertex, FullCell> TDS;
typedef CGAL::Triangulation<Traits, TDS> Triangulation;
typedef Triangulation::Facet Facet;
typedef TDS::Vertex_iterator VertexIter;

auto compare_prio = [](auto left, auto right) { return left.data().prio() > right.data().prio(); };

typedef std::priority_queue<TDS::Full_cell, std::vector<TDS::Full_cell>, decltype(compare_prio)>
    CellContainer;

namespace c = std::chrono;

struct ImportantMetric {
  size_t metric;
  double slack;
};

typedef std::array<bool, DIMENSION> Important;
typedef std::array<double, DIMENSION> Slack;

class EnumerateOptimals {
  public:
  typedef std::vector<std::pair<size_t, size_t>> Edges;

  private:
  Graph* g;
  Triangulation tri { DIMENSION };
  std::vector<Route> routes;
  std::vector<Config> configs;
  std::map<std::pair<size_t, size_t>, double> similarities;
  double maxOverlap;
  size_t maxRoutes;
  Dijkstra d;
  Dijkstra::ScalingFactor factor;
  Important important;
  Slack slack;

  double compare(size_t i, size_t j);
  Config findConfig(const TDS::Full_cell& f);
  void addToTriangulation();
  void includeConvexHullCells(CellContainer& cont);
  std::vector<size_t> extract_independent_set(const std::vector<size_t>& vertices, bool ilp);
  std::tuple<std::vector<size_t>, EnumerateOptimals::Edges> vertex_ids_and_edges();
  exclusion_set create_exclusion_set_from_important_metrics(NodePos s, NodePos t);
  void run_not_important_metrics(NodePos s, NodePos t);

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

#endif /* ENUMERATE_OPTIMALS_H */
