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
#include "glpk.h"
#include "ilp_independent_set.hpp"
#include "routeComparator.hpp"

#include <CGAL/Epick_d.h>
#include <CGAL/Triangulation.h>
#include <CGAL/Triangulation_ds_full_cell.h>
#include <CGAL/point_generators_d.h>
#include <CGAL/random_selection.h>
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
  FullCellId()
      : id_(currentId++)
  {
    alive_.push_back(1);
    checked_.push_back(false);
    prio_.push_back(-1.0);
  }

  FullCellId(const FullCellId& other)
      : id_(other.id_)
  {
    alive_[id_]++;
  }

  FullCellId& operator=(const FullCellId& rhs)
  {
    // Check for self-assignment!
    if (this == &rhs)
      return *this;

    alive_[id_]--;
    id_ = rhs.id_;
    alive_[id_]++;
    return *this;
  }

  FullCellId& operator=(const FullCellId&& rhs)
  {
    *this = rhs;
    return *this;
  }

  FullCellId(FullCellId&& other)
  {
    id_ = other.id_;
    alive_[id_]++;
  }

  ~FullCellId() { alive_[id_]--; }

  bool alive() const { return alive_[id_] > 1; }
  bool checked() const { return checked_[id_]; }
  void checked(bool check) { checked_[id_] = check; }
  size_t id() const { return id_; }
  double prio() const { return prio_[id_]; }
  void prio(double p) const { prio_[id_] = p; }

  bool operator==(const FullCellId& other) const { return id_ == other.id_; }
};

struct VertexData {
  size_t id;
};

int FullCellId::currentId = 0;
std::vector<int> FullCellId::alive_ = std::vector<int>();
std::vector<bool> FullCellId::checked_ = std::vector<bool>();
std::vector<double> FullCellId::prio_ = std::vector<double>();

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

class EnumerateOptimals {

  Dijkstra d;
  Triangulation tri{ DIMENSION };
  std::vector<Route> routes;
  std::vector<Config> configs;
  std::map<std::pair<size_t, size_t>, double> similarities;
  double maxOverlap;
  size_t maxRoutes;

  double compare(size_t i, size_t j)
  {
    if (i > j) {
      std::swap(i, j);
    }
    std::pair index = { i, j };
    if (similarities.count(index) > 0) {
      return similarities[index];
    }
    auto similarity = calculateSharing(routes[i], routes[j]);
    similarities[index] = similarity;
    return similarity;
  }

  typedef std::unique_ptr<glp_prob, decltype(&glp_delete_prob)> lp_ptr;

  void defineVariables(const lp_ptr& lp)
  {
    glp_add_cols(lp.get(), DIMENSION + 1);
    for (size_t i = 0; i < DIMENSION; ++i) {
      glp_set_col_bnds(lp.get(), i + 1, GLP_DB, 0, 1);
      glp_set_col_kind(lp.get(), i + 1, GLP_CV);
      glp_set_obj_coef(lp.get(), i + 1, 0);
    }
    glp_set_col_bnds(lp.get(), DIMENSION + 1, GLP_LO, 0, 0);
    glp_set_col_kind(lp.get(), DIMENSION + 1, GLP_CV);
    glp_set_obj_coef(lp.get(), DIMENSION + 1, 0);
  }

  void addConstraints(const lp_ptr& lp, const TDS::Full_cell& f)
  {
    // Cost * alpha - V = 0 for all paths of f
    std::vector ind(1, 0);
    for (size_t i = 1; i <= DIMENSION + 1; ++i) {
      ind.push_back(i);
    }
    size_t constraint_count = 0;
    for (auto vertex = f.vertices_begin(); vertex != f.vertices_end(); ++vertex) {
      if (*vertex == TDS::Vertex_const_handle() || tri.is_infinite(*vertex)) {
        continue;
      }
      int slack = glp_add_cols(lp.get(), 1);
      glp_set_col_bnds(lp.get(), slack, GLP_LO, 0, 0);
      glp_set_col_kind(lp.get(), slack, GLP_CV);
      glp_set_obj_coef(lp.get(), slack, 1);

      int row = glp_add_rows(lp.get(), 1);
      std::vector val(1, 0.0);
      auto& p = (*vertex)->point();
      for (auto comp = p.cartesian_begin(); comp != p.cartesian_end(); ++comp) {
        val.push_back(*comp);
      }
      val.push_back(-1);

      ind.push_back(slack);
      val.push_back(-1);

      glp_set_row_bnds(lp.get(), row, GLP_FX, 0.0, 0.0);
      glp_set_mat_row(lp.get(), row, ind.size() - 1, &ind[0], &val[0]);

      ind.pop_back();
      ++constraint_count;
    }

    if (constraint_count <= 1) {
      throw std::runtime_error("Could not create enough constraints");
    }

    // sum(alpha) = 1
    std::vector val(DIMENSION + 2, 1.0);
    val[0] = 0.0;
    val.back() = 0.0;

    int row = glp_add_rows(lp.get(), 1);
    glp_set_row_bnds(lp.get(), row, GLP_FX, 1.0, 1.0);
    glp_set_mat_row(lp.get(), row, DIMENSION + 1, &ind[0], &val[0]);
  }

  Config findConfig(const TDS::Full_cell& f)
  {
    lp_ptr lp(glp_create_prob(), glp_delete_prob);
    glp_set_obj_dir(lp.get(), GLP_MIN);
    defineVariables(lp);
    addConstraints(lp, f);

    glp_smcp params;
    glp_init_smcp(&params);
    params.msg_lev = GLP_MSG_OFF;
    params.presolve = GLP_ON;
    int simplex = glp_simplex(lp.get(), &params);
    int lp_status = glp_get_status(lp.get());
    if (simplex == 0 && (lp_status == GLP_OPT || lp_status == GLP_FEAS)) {
      std::vector values(DIMENSION, 0.0);
      for (size_t i = 0; i < DIMENSION; ++i) {
        values[i] = glp_get_col_prim(lp.get(), i + 1);
      }
      return values;
    } else {
      throw std::runtime_error("no suitable config found. Simplex: " + std::to_string(simplex)
          + " lp: " + std::to_string(lp_status));
    }
  }

  void addToTriangulation()
  {
    auto vertId = routes.size() - 1;
    auto& routeCosts = routes[vertId].costs;
    Triangulation::Point p(DIMENSION, &routeCosts.values[0], &routeCosts.values[DIMENSION]);
    auto vertex = tri.insert(p);
    auto& vertexdata = vertex->data();
    vertexdata.id = vertId;
  }

  void includeConvexHullCells(CellContainer& cont)
  {
    std::vector<TDS::Full_cell_handle> handles;
    tri.incident_full_cells(tri.infinite_vertex(), std::back_inserter(handles));
    for (size_t i = 0; i < handles.size(); ++i) {
      TDS::Full_cell& c = *handles[i];
      if (c.data().checked()) {
        continue;
      }

      if (c.data().prio() < 0.0) {
        std::vector<TDS::Vertex_handle> vertices;
        for (auto v = c.vertices_begin(); v != c.vertices_end(); ++v) {
          auto& vertex = *v;
          if (vertex == TDS::Vertex_const_handle() || tri.is_infinite(vertex)) {
            continue;
          }
          vertices.push_back(vertex);
        }

        auto result = 0;
        for (size_t i = 0; i < routes.size(); ++i) {
          for (auto& vertex : vertices) {
            auto vertId = vertex->data().id;
            if (vertId != i && compare(i, vertId) > maxOverlap) {
              ++result;
            }
          }
        }
        c.data().prio(result);
      }
      cont.emplace(c);
    }
  }

  public:
  size_t enumeration_time;
  size_t recommendation_time;

  EnumerateOptimals(Graph& g, double maxOverlap, size_t maxRoutes)
      : d(g.createDijkstra())
      , maxOverlap(maxOverlap)
      , maxRoutes(maxRoutes)
  {
  }

  EnumerateOptimals(Dijkstra& d, double maxOverlap, size_t maxRoutes)
      : d(d)
      , maxOverlap(maxOverlap)
      , maxRoutes(maxRoutes)
  {
  }

  std::tuple<std::vector<Route>, std::vector<Config>> find(NodePos s, NodePos t)
  {
    namespace c = std::chrono;

    auto start = c::high_resolution_clock::now();
    routes.clear();
    configs.clear();
    similarities.clear();
    tri.clear();

    Config conf(std::vector(DIMENSION, 1.0 / DIMENSION));
    auto route = d.findBestRoute(s, t, conf);
    if (!route) {
      throw std::runtime_error(
          "No route found from " + std::to_string(s) + " to " + std::to_string(t));
    }

    routes.push_back(std::move(*route));
    configs.push_back(std::move(conf));
    addToTriangulation();

    Dijkstra::ScalingFactor factor;
    for (size_t i = 0; i < DIMENSION; ++i) {
      Config conf(std::vector(DIMENSION, 0.0));
      conf.values[i] = 1.0;

      auto route = d.findBestRoute(s, t, conf);
      factor[i] = route->costs.values[i];
      routes.push_back(std::move(*route));
      configs.push_back(std::move(conf));
      addToTriangulation();
    }
    double maxValue = *std::max_element(&factor[0], &factor[DIMENSION]);

    for (double& value : factor) {
      value = maxValue / value;
    }

    CellContainer q(compare_prio);
    bool workToDo = true;
    while (workToDo) {
      includeConvexHullCells(q);
      workToDo = false;
      while (!q.empty() && tri.number_of_vertices() < maxRoutes) {
        auto f = q.top();
        q.pop();
        FullCellId& cellData = const_cast<FullCellId&>(f.data());

        if (!cellData.alive() || cellData.checked()) {
          continue;
        }
        cellData.checked(true);

        try {
          Config conf = findConfig(f);
          auto route = d.findBestRoute(s, t, conf);
          TDS::Vertex_handle v;
          for (auto vertex = f.vertices_begin(); vertex != f.vertices_end(); ++vertex) {
            if (*vertex == TDS::Vertex_const_handle() || tri.is_infinite(*vertex)) {
              continue;
            } else {
              v = *vertex;
              break;
            }
          }

          routes.push_back(std::move(*route));
          configs.push_back(std::move(conf));

          bool include = true;
          for (size_t i = 0; i < routes.size() - 2; ++i) {
            auto lastRoute = routes.size() - 1;
            if (compare(i, lastRoute) == 1.0) {
              include = false;
              break;
            }
          }

          if (include) {
            addToTriangulation();
            workToDo = true;
          }

        } catch (std::runtime_error& e) {
        }
      }
    }

    auto end = c::high_resolution_clock::now();

    enumeration_time = c::duration_cast<c::milliseconds>(end - start).count();

    start = c::high_resolution_clock::now();
    std::vector<size_t> vertices;

    if (tri.number_of_vertices() >= DIMENSION) {
      typedef Triangulation::Face Face;
      typedef std::vector<Face> Faces;
      Faces ch_edges;
      std::back_insert_iterator<Faces> out(ch_edges);
      tri.tds().incident_faces(tri.infinite_vertex(), 1, out);

      vertices.reserve(ch_edges.size());
      for (auto& f : ch_edges) {
        TDS::Vertex_handle v = f.vertex(0);
        if (tri.is_infinite(v)) {
          v = f.vertex(1);
        }
        vertices.push_back(v->data().id);
      }
    } else {
      for (TDS::Vertex_iterator vertex = tri.vertices_begin(); vertex != tri.vertices_end();
           ++vertex) {
        if (vertex == TDS::Vertex_handle() || tri.is_infinite(*vertex)) {
          continue;
        }
        vertices.push_back(vertex->data().id);
      }
    }

    std::vector<size_t> independent_set;
    if (maxOverlap < 1.0) {
      std::vector<std::pair<size_t, size_t>> edges;

      for (size_t i = 0; i < vertices.size(); ++i) {
        for (size_t j = i + 1; j < vertices.size(); ++j) {
          if (compare(vertices[i], vertices[j]) >= maxOverlap) {
            edges.emplace_back(i, j);
          }
        }
      }
      independent_set = find_independent_set(vertices.size(), edges);
    } else {
      for (size_t i = 0; i < vertices.size(); ++i) {
        independent_set.push_back(i);
      }
    }
    end = c::high_resolution_clock::now();
    recommendation_time = c::duration_cast<c::milliseconds>(end - start).count();

    std::vector<Route> routes;
    routes.reserve(independent_set.size());
    std::vector<Config> configs;
    configs.reserve(independent_set.size());

    for (auto id : independent_set) {
      routes.push_back(this->routes[vertices[id]]);

      auto c = this->configs[vertices[id]];
      for (size_t i = 0; i < DIMENSION; ++i) {
        c.values[i] /= factor[i];
      }
      auto sum = std::accumulate(&c.values[0], &c.values[DIMENSION], 0.0);

      for (size_t i = 0; i < DIMENSION; ++i) {
        c.values[i] /= sum;
      }

      configs.push_back(std::move(c));
    }

    return { routes, configs };
  }

  size_t found_route_count() const { return routes.size(); }
  size_t vertex_count() const { return tri.number_of_vertices(); }
};

#endif /* ENUMERATE_OPTIMALS_H */
