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

#ifndef PRIO_POLICY_H
#define PRIO_POLICY_H

#include "cgaltypes.hpp"

template <int Dim> struct FacetPrioPolicy {
  using Full_cell = typename CgalTypes<Dim>::TDS::Full_cell;
  using Route = Route<Dim>;
  using Config = Config<Dim>;
  using Vertex_handle = typename CgalTypes<Dim>::TDS::Vertex_handle;

  double calc_prio(const std::vector<Vertex_handle>&) { return 1.0; };
  void set_pointers(std::vector<Route>* routes, std::vector<Config>* configs,
      std::function<double(size_t, size_t)> compare)
  {
    this->routes = routes;
    this->configs = configs;
    this->compare = compare;
  }

  protected:
  std::vector<Route>* routes = nullptr;
  std::vector<Config>* configs = nullptr;
  std::function<double(size_t, size_t)> compare;
};

template <int Dim> struct SimilarityPrioPolicy : public FacetPrioPolicy<Dim> {
  using Base = FacetPrioPolicy<Dim>;
  using Full_cell = typename Base::Full_cell;
  using Vertex_handle = typename Base::Vertex_handle;

  SimilarityPrioPolicy(double max_overlap)
      : max_overlap(max_overlap) {};

  double calc_prio(const std::vector<Vertex_handle>& vertices)
  {
    auto result = 0;
    for (size_t i = 0; i < Base::routes->size(); ++i) {
      for (auto& vertex : vertices) {
        auto vertId = vertex->data().id;
        if (vertId != i && Base::compare(i, vertId) > max_overlap) {
          ++result;
        }
      }
    }

    return result;
  };

  private:
  double max_overlap;
};
#endif /* PRIO_POLICY_H */
