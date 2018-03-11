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

#include "triangleexplorer.hpp"

AlternativeRoutes explore(Graph& g, NodePos from, NodePos to)
{
  PosVector lengthVec{ { 1, 0, 0 } };
  PosVector heightVec{ { 0, 1, 0 } };
  PosVector unsuitVec{ { 0, 0, 1 } };

  auto d = g.createDijkstra();
  std::vector<Route> routes;

  std::vector<Triangle> triangles;

  routes.push_back(d.findBestRoute(from, to, lengthVec).value());
  Point length(lengthVec, routes.size() - 1);

  routes.push_back(d.findBestRoute(from, to, heightVec).value());
  Point height(heightVec, routes.size() - 1);

  routes.push_back(d.findBestRoute(from, to, unsuitVec).value());
  Point unsuit(unsuitVec, routes.size() - 1);

  double lengthUnsuit
      = DiscreteFrechet(routes[length.routeIndex], routes[unsuit.routeIndex], g).calculate();
  double lengthHeight
      = DiscreteFrechet(routes[length.routeIndex], routes[height.routeIndex], g).calculate();
  double unsuitHeight
      = DiscreteFrechet(routes[unsuit.routeIndex], routes[height.routeIndex], g).calculate();

  std::optional<double> maxFrechet;
  double lengthFrechet = lengthUnsuit + lengthHeight;
  double heightFrechet = lengthHeight + unsuitHeight;
  double unsuitFrechet = lengthUnsuit + unsuitHeight;
  while (true) {
    auto& t = triangles.emplace_back(length, height, unsuit);
    auto middleVec = t.calculateWeightedMiddle(lengthFrechet, heightFrechet, unsuitFrechet);
    routes.push_back(d.findBestRoute(from, to, middleVec).value());
    Config c = middleVec;
    std::cout << "middle = " << c.length << " / " << c.height << " / " << c.unsuitability << '\n';

    Point middle(middleVec, routes.size() - 1);

    lengthFrechet = DiscreteFrechet(routes[length.routeIndex], routes[3], g).calculate();
    heightFrechet = DiscreteFrechet(routes[height.routeIndex], routes[3], g).calculate();
    unsuitFrechet = DiscreteFrechet(routes[unsuit.routeIndex], routes[3], g).calculate();
    auto newMaxFrechet = std::max({ lengthFrechet, heightFrechet, unsuitFrechet });
    if (maxFrechet) {
      std::cout << "max: " << *maxFrechet << " new: " << newMaxFrechet << '\n';
    } else {
      std::cout << "new: " << newMaxFrechet << '\n';
    }
    if (maxFrechet && std::abs(newMaxFrechet - *maxFrechet) < 10) {
      break;
    }
    maxFrechet = newMaxFrechet;

    if (lengthFrechet > heightFrechet) {
      if (unsuitFrechet > heightFrechet) {
        height = middle;
      } else {
        unsuit = middle;
      }
    } else {
      if (unsuitFrechet > lengthFrechet) {
        length = middle;
      } else {
        unsuit = middle;
      }
    }
  }
  return AlternativeRoutes(
      triangles[0].calculateMiddle(), routes[3], triangles.back().getMiddle(), routes.back());
}
