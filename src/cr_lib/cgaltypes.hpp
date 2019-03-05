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

#ifndef CGALTYPES_H
#define CGALTYPES_H

#include <CGAL/Epick_d.h>
#include <CGAL/Triangulation.h>
#include <CGAL/Triangulation_ds_full_cell.h>
#include <CGAL/point_generators_d.h>
#include <CGAL/random_selection.h>

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

template <int Dim> struct CgalTypes {
  using CgalDim = CGAL::Dimension_tag<Dim>;
  using Traits = CGAL::Epick_d<CgalDim>;
  using Vertex = CGAL::Triangulation_vertex<Traits, VertexData>;
  using FullCell = CGAL::Triangulation_full_cell<Traits, FullCellId>;
  using TDS = CGAL::Triangulation_data_structure<CgalDim, Vertex, FullCell>;
  using Triangulation = CGAL::Triangulation<Traits, TDS>;
  using Facet = typename Triangulation::Facet;
  using VertexIter = typename TDS::Vertex_iterator;
};

#endif /* CGALTYPES_H */
