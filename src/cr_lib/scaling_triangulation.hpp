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
#ifndef SCALING_TRIANGULATION_H
#define SCALING_TRIANGULATION_H

#include "dijkstra.hpp"
#include "posvector.hpp"

struct TriPoint {
  PosVector p;
  Route r;
  bool selected;
};

struct TriTriangle {
  size_t point1;
  size_t point2;
  size_t point3;

  bool noChildren;
  bool noMoreRoutes;
};

struct TriangulationResult {
  std::vector<TriPoint> points;
  std::vector<TriTriangle> triangles;
  size_t explore_time;
  size_t recommendation_time;
};

TriangulationResult scaledTriangulation(Dijkstra& d, NodePos from, NodePos to, size_t maxSplits,
    std::optional<size_t> maxLevel, bool splitByLevel, double maxOverlap);

#endif /* SCALING_TRIANGULATION_H */
