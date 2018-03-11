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
#ifndef TRIANGLEEXPLORER_H
#define TRIANGLEEXPLORER_H

#include "dijkstra.hpp"
#include "graph.hpp"
#include "posvector.hpp"
#include "routeComparator.hpp"

const double oneThird = 1.0 / 3.0;

struct Point {
  PosVector position;
  size_t routeIndex;

  Point(PosVector pos, size_t routeIndex)
      : position(std::move(pos))
      , routeIndex(routeIndex)
  {
  }
};

class Triangle {
  public:
  Triangle(Point a, Point b, Point c)
      : a(std::move(a))
      , b(std::move(b))
      , c(std::move(c))
  {
  }

  PosVector calculateMiddle() const { return (a.position + b.position + c.position) * oneThird; }
  PosVector calculateWeightedMiddle(double aWeight, double bWeight, double cWeight)
  {
    middle = (a.position * aWeight + b.position * bWeight + c.position * cWeight)
        * (1 / (aWeight + bWeight + cWeight));
    return *middle;
  }

  PosVector getMiddle() const { return *middle; }

  private:
  Point a;
  Point b;
  Point c;
  std::optional<PosVector> middle;
};

struct AlternativeRoutes {
  Config config1;
  Route route1;
  Config config2;
  Route route2;

  AlternativeRoutes(
      const Config& config1, const Route& route1, const Config& config2, const Route& route2)
      : config1(config1)
      , route1(route1)
      , config2(config2)
      , route2(route2)
  {
  }
};

AlternativeRoutes explore(Graph& g, NodePos from, NodePos to);

#endif /* TRIANGLEEXPLORER_H */
