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

double calculateSharing(const Route& referenceRoute, const Route& otherRoute);

class DiscreteFrechet {
  public:
  DiscreteFrechet(const Route& reference, const Route& other, const Graph& g);
  virtual ~DiscreteFrechet() = default;

  double calculate();

  private:
  std::vector<std::vector<double>> ca;
  std::vector<const Node*> refNodes;
  std::vector<const Node*> otherNodes;

  double c(int i, int j);
};

#endif /* ROUTECOMPARATOR_H */
