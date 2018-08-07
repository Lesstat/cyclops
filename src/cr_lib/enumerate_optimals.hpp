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

#include <CGAL/Epick_d.h>
#include <CGAL/Triangulation.h>
#include <CGAL/Triangulation_ds_full_cell.h>
#include <CGAL/point_generators_d.h>
#include <CGAL/random_selection.h>
#include <queue>

class FullCellId {
  private:
  static int currentId;
  static std::vector<int> alive_;
  static std::vector<bool> checked_;

  int id_;

  public:
  FullCellId()
      : id_(currentId++)
  {
    alive_.push_back(1);
    checked_.push_back(false);
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

  bool operator==(const FullCellId& other) const { return id_ == other.id_; }
};

struct VertexData {
  size_t id;
};

int FullCellId::currentId = 0;
std::vector<int> FullCellId::alive_ = std::vector<int>();
std::vector<bool> FullCellId::checked_ = std::vector<bool>();

typedef CGAL::Dimension_tag<DIMENSION> Dim;
typedef CGAL::Epick_d<Dim> Traits;
typedef CGAL::Triangulation_vertex<Traits, VertexData> Vertex;
typedef CGAL::Triangulation_full_cell<Traits, FullCellId> FullCell;
typedef CGAL::Triangulation_data_structure<Dim, Vertex, FullCell> TDS;
typedef CGAL::Triangulation<Traits, TDS> Triangulation;
typedef Triangulation::Facet Facet;
typedef TDS::Vertex_iterator VertexIter;

typedef std::unordered_map<int, TDS::Full_cell> CellSet;

class EnumerateOptimals {

  Dijkstra d;
  Triangulation tri{ DIMENSION };
  std::vector<Route> routes;
  std::vector<Config> configs;

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
    glp_set_obj_coef(lp.get(), DIMENSION + 1, 1);
  }

  void addConstraints(const lp_ptr& lp, const TDS::Full_cell& f)
  {

    // Cost * alpha - V = 0 for all paths of f
    std::vector ind(1, 0);
    for (size_t i = 1; i <= DIMENSION + 1; ++i) {
      ind.push_back(i);
    }

    for (auto vertex = f.vertices_begin(); vertex != f.vertices_end(); ++vertex) {
      if (tri.is_infinite(*vertex)) {
        continue;
      }
      int row = glp_add_rows(lp.get(), 1);
      std::vector val(1, 0.0);
      auto& p = (*vertex)->point();
      for (auto comp = p.cartesian_begin(); comp != p.cartesian_end(); ++comp) {
        val.push_back(*comp);
      }
      val.push_back(-1);
      glp_set_row_bnds(lp.get(), row, GLP_FX, 0.0, 0.0);
      glp_set_mat_row(lp.get(), row, DIMENSION + 1, &ind[0], &val[0]);
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
      throw std::runtime_error("no suitable config found");
    }
  }
  void addToTriangulation(Route&& r, Config&& c)
  {
    Triangulation::Point p(DIMENSION, &r.costs.values[0], &r.costs.values[DIMENSION]);
    auto vertex = tri.insert(p);
    auto& vertexdata = vertex->data();
    routes.push_back(std::move(r));
    configs.push_back(std::move(c));
    vertexdata.id = routes.size() - 1;
  }

  void includeConvexHullCells(CellSet& set)
  {
    std::vector<TDS::Full_cell_handle> handles;
    tri.incident_full_cells(tri.infinite_vertex(), std::back_inserter(handles));
    for (size_t i = 0; i < handles.size(); ++i) {
      TDS::Full_cell& c = *handles[i];
      set.emplace(std::make_pair(c.data().id(), c));
    }
  }

  public:
  EnumerateOptimals(Graph& g)
      : d(g.createDijkstra())
  {
  }

  std::tuple<std::vector<Route>, std::vector<Config>> find(NodePos s, NodePos t)
  {

    Config c(std::vector(DIMENSION, 1.0 / DIMENSION));
    auto route = d.findBestRoute(s, t, c);
    if (!route) {
      throw std::runtime_error(
          "No route found from " + std::to_string(s) + " to " + std::to_string(t));
    }
    addToTriangulation(std::move(*route), std::move(c));
    for (size_t i = 0; i < DIMENSION; ++i) {
      Config c(std::vector(DIMENSION, 0.0));
      c.values[i] = 1.0;

      auto route = d.findBestRoute(s, t, c);
      addToTriangulation(std::move(*route), std::move(c));
    }

    CellSet q;
    bool workToDo = true;
    while (workToDo) {
      includeConvexHullCells(q);
      workToDo = false;
      std::vector<Cost> newPoints;
      std::vector<int> deadCells;
      for (auto it = q.begin(); it != q.end(); ++it) {
        auto& f = it->second;
        FullCellId& cellData = const_cast<FullCellId&>(f.data());

        if (!cellData.alive()) {
          deadCells.push_back(cellData.id());
          continue;
        }
        if (cellData.checked()) {
          continue;
        }
        cellData.checked(true);

        try {
          Config c = findConfig(f);
          auto route = d.findBestRoute(s, t, c);
          auto v = f.vertex(0);
          if (tri.is_infinite(v)) {
            v = f.vertex(1);
          }

          Cost& cellCost = routes[v->data().id].costs;
          if (route->costs * c < cellCost * c) {
            addToTriangulation(std::move(*route), std::move(c));
            workToDo = true;
          }

        } catch (std::runtime_error& e) {
          continue;
        }
      }

      for (auto cell : deadCells) {
        q.erase(cell);
      }
    }

    typedef Triangulation::Face Face;
    typedef std::vector<Face> Faces;
    Faces edges;
    std::back_insert_iterator<Faces> out(edges);
    tri.tds().incident_faces(tri.infinite_vertex(), 1, out);

    std::vector<Route> routes;
    routes.reserve(edges.size());
    std::vector<Config> configs;
    configs.reserve(edges.size());

    for (auto& edge : edges) {
      auto vertex = edge.vertex(0);
      if (tri.is_infinite(*vertex)) {
        vertex = edge.vertex(1);
      }
      auto& vertexId = vertex->data().id;

      routes.push_back(this->routes[vertexId]);
      configs.push_back(this->configs[vertexId]);
    }
    return { routes, configs };
  }
};

#endif /* ENUMERATE_OPTIMALS_H */
