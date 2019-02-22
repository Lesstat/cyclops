/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2018  Florian Barth

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
#ifndef ROUTECOMPARATOR_H
#define ROUTECOMPARATOR_H

#include "dijkstra.hpp"

template <int Dim>
double calculateSharing(const Route<Dim>& referenceRoute, const Route<Dim>& otherRoute);

template <int Dim> class DiscreteFrechet {
  public:
  using Route = Route<Dim>;
  using Graph = Graph<Dim>;

  DiscreteFrechet(const Route& reference, const Route& other, const Graph& g);
  virtual ~DiscreteFrechet() = default;

  double calculate();

  private:
  std::vector<std::vector<double>> ca;
  std::vector<Node> reference;
  std::vector<Node> other;

  double c(int i, int j);
};

#include "routeComparator.inc"
#endif /* ROUTECOMPARATOR_H */
