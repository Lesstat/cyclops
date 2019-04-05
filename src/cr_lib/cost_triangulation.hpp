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
#ifndef COST_TRIANGULATION_H
#define COST_TRIANGULATION_H

#include "cgaltypes.hpp"
#include "graph.hpp"

template <int Dim, class Derived> class CostTriangulation {
  using Cost = Cost<Dim>;

  using CgalDim = typename CgalTypes<Dim>::CgalDim;
  using Traits = typename CgalTypes<Dim>::Traits;
  using Vertex = typename CgalTypes<Dim>::Vertex;
  using FullCell = typename CgalTypes<Dim>::FullCell;
  using TDS = typename CgalTypes<Dim>::TDS;
  using Triangulation = typename CgalTypes<Dim>::Triangulation;
  using Facet = typename CgalTypes<Dim>::Facet;
  using VertexIter = typename CgalTypes<Dim>::VertexIter;
  using Point = typename Triangulation::Point;
  using Route = Route<Dim>;

  Triangulation tri { Dim };

  public:
  void tri_clear() { tri.clear(); }

  void add_route(const Cost& c, size_t id)
  {
    Point p(Dim, &c.values[0], &c.values[Dim]);
    auto vertex = tri.insert(p);
    auto& vertexdata = vertex->data();
    vertexdata.id = id;
  }

  template <class Container> void get_convex_hull_cells(Container& cont)
  {
    Derived* base = static_cast<Derived*>(this);

    std::vector<typename TDS::Full_cell_handle> handles;
    tri.incident_full_cells(tri.infinite_vertex(), std::back_inserter(handles));
    for (size_t i = 0; i < handles.size(); ++i) {
      typename TDS::Full_cell& cell = *handles[i];
      if (cell.data().checked()) {
        continue;
      }

      auto current_cell_vertices = cell_vertices(cell);
      if (base->exclude_facet(current_cell_vertices)) {
        cell.data().checked(true);
        continue;
      }

      if (cell.data().prio() < 0.0) {
        double prio = base->calc_prio(current_cell_vertices);
        cell.data().prio(prio);
      }
      cont.emplace_back(cell);
    }
  }

  std::vector<typename TDS::Vertex_handle> cell_vertices(const typename TDS::Full_cell& f)
  {
    std::vector<typename TDS::Vertex_handle> vertices;

    for (auto v = f.vertices_begin(); v != f.vertices_end(); ++v) {
      typename TDS::Vertex_handle vert = *v;
      if (vert == typename TDS::Vertex_handle() || tri.is_infinite(vert))
        continue;
      vertices.push_back(vert);
    }

    return vertices;
  }

  std::tuple<std::vector<size_t>, std::vector<std::pair<size_t, size_t>>> vertex_ids_and_edges()
  {
    typedef typename Triangulation::Face Face;
    typedef std::vector<Face> Faces;

    using Edges = std::vector<std::pair<size_t, size_t>>;

    std::vector<size_t> vertices;
    Edges edges;

    auto get_edges = [&edges, this](auto v) {
      Faces adjacent_edges;
      tri.tds().incident_faces(v, 1, std::back_inserter(adjacent_edges));
      for (auto& f : adjacent_edges) {
        auto v0 = f.vertex(0);
        auto v1 = f.vertex(1);
        if (tri.is_infinite(v0) || tri.is_infinite(v1)) {
          continue;
        }
        auto id0 = v0->data().id;
        auto id1 = v1->data().id;
        if (id0 > id1) {
          std::swap(id0, id1);
        }
        edges.emplace_back(id0, id1);
      }
    };

    Derived* base = static_cast<Derived*>(this);

    if (tri.number_of_vertices() >= Dim) {
      Faces ch_edges;
      std::back_insert_iterator<Faces> out(ch_edges);
      tri.tds().incident_faces(tri.infinite_vertex(), 1, out);

      vertices.reserve(ch_edges.size());
      for (auto& f : ch_edges) {
        typename TDS::Vertex_handle v = f.vertex(0);
        if (tri.is_infinite(v)) {
          v = f.vertex(1);
        }
        if (base->exclude_route(base->route(v->data().id))) {
          continue;
        }
        vertices.push_back(v->data().id);
        get_edges(v);
      }
    } else {
      for (typename TDS::Vertex_iterator vertex = tri.vertices_begin();
           vertex != tri.vertices_end(); ++vertex) {
        if (vertex == typename TDS::Vertex_handle() || tri.is_infinite(*vertex)) {
          continue;
        }
        if (base->exclude_route(base->route(vertex->data().id))) {
          continue;
        }
        vertices.push_back(vertex->data().id);
        get_edges(vertex);
      }
    }
    std::sort(edges.begin(), edges.end());
    auto last = std::unique(edges.begin(), edges.end());
    edges.erase(last, edges.end());

    return { vertices, edges };
  }
};
#endif /* COST_TRIANGULATION_H */
