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

RouteExplorer::RouteExplorer(Graph* g, NodePos from, NodePos to)
    : g(g)
    , from(from)
    , to(to)
    , d(g->createDijkstra())
{
}

Point RouteExplorer::createPoint(const PosVector& pos)
{
  routes.push_back(d.findBestRoute(from, to, pos).value());
  return Point(pos, routes.size() - 1);
}

AlternativeRoutes RouteExplorer::weightedExplore()
{
  routes.clear();

  PosVector lengthVec{ { 1, 0, 0 } };
  PosVector heightVec{ { 0, 1, 0 } };
  PosVector unsuitVec{ { 0, 0, 1 } };

  std::vector<Triangle> triangles;

  auto length = createPoint(lengthVec);
  auto height = createPoint(heightVec);
  auto unsuit = createPoint(unsuitVec);

  double lengthUnsuit
      = DiscreteFrechet(routes[length.routeIndex], routes[unsuit.routeIndex], *g).calculate();
  double lengthHeight
      = DiscreteFrechet(routes[length.routeIndex], routes[height.routeIndex], *g).calculate();
  double unsuitHeight
      = DiscreteFrechet(routes[unsuit.routeIndex], routes[height.routeIndex], *g).calculate();

  std::optional<double> maxFrechet;
  double lengthFrechet = lengthUnsuit + lengthHeight;
  double heightFrechet = lengthHeight + unsuitHeight;
  double unsuitFrechet = lengthUnsuit + unsuitHeight;

  auto replaceSmallestCornerWith = [&](const Point& middle) {
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
  };

  auto searchAlternative = [&](const size_t& index) {
    while (true) {
      auto& t = triangles.emplace_back(length, height, unsuit);
      auto middleVec = t.calculateWeightedMiddle(lengthFrechet, heightFrechet, unsuitFrechet);
      Config c = middleVec;
      std::cout << "middle = " << c.length << " / " << c.height << " / " << c.unsuitability << '\n';

      auto middle = createPoint(middleVec);

      lengthFrechet = DiscreteFrechet(routes[length.routeIndex], routes[index], *g).calculate();
      heightFrechet = DiscreteFrechet(routes[height.routeIndex], routes[index], *g).calculate();
      unsuitFrechet = DiscreteFrechet(routes[unsuit.routeIndex], routes[index], *g).calculate();
      std::cout << "lengthF: " << lengthFrechet << ", heightF: " << heightFrechet
                << ", unsuitF: " << unsuitFrechet << '\n';

      auto newMaxFrechet = std::max({ lengthFrechet, heightFrechet, unsuitFrechet });
      if (maxFrechet) {
        std::cout << "max: " << *maxFrechet << " new: " << newMaxFrechet << '\n';
      } else {
        std::cout << "new: " << newMaxFrechet << '\n';
      }

      if (maxFrechet && newMaxFrechet <= maxFrechet) {
        break;
      }
      maxFrechet = newMaxFrechet;

      replaceSmallestCornerWith(middle);
    }
  };

  searchAlternative(3);

  std::cout << "-------- Searching for second Route --------------" << '\n';

  length = triangles.front().a();
  height = triangles.front().b();
  unsuit = triangles.front().c();

  auto& firstRoute = routes.back();

  lengthFrechet = DiscreteFrechet(routes[length.routeIndex], firstRoute, *g).calculate();
  heightFrechet = DiscreteFrechet(routes[height.routeIndex], firstRoute, *g).calculate();
  unsuitFrechet = DiscreteFrechet(routes[unsuit.routeIndex], firstRoute, *g).calculate();

  auto first = Point(triangles.back().getMiddle(), routes.size() - 1);

  searchAlternative(routes.size() - 1);

  return AlternativeRoutes(first.position, firstRoute, triangles.back().getMiddle(), routes.back());
}

// Point createRouteForPoint(onst PosVector& pos, std::vector<Route> & routes, Dijkstra& d){}

AlternativeRoutes RouteExplorer::exploreGreatestDistance()
{
  PosVector lengthVec{ { 1, 0, 0 } };
  PosVector heightVec{ { 0, 1, 0 } };
  PosVector unsuitVec{ { 0, 0, 1 } };

  std::vector<Triangle> triangles;

  routes.clear();

  auto length = createPoint(lengthVec);
  auto height = createPoint(heightVec);
  auto unsuit = createPoint(unsuitVec);

  auto main = triangles.emplace_back(length, height, unsuit);
  PosVector middleVec = main.calculateMiddle();
  auto middle = createPoint(middleVec);
  std::cout << "Exploring" << '\n';
  while (true) {
    middleVec = main.calculateMiddle();
    middle = createPoint(middleVec);

    Triangle noLength(middle, main.b(), main.c());
    Triangle noHeight(main.a(), middle, main.c());
    Triangle noUnsuit(main.a(), main.b(), middle);

    auto noLengthVec = noLength.calculateMiddle();
    auto noHeightVec = noHeight.calculateMiddle();
    auto noUnsuitVec = noUnsuit.calculateMiddle();

    auto noLengthMiddle = createPoint(noLengthVec);
    auto noHeightMiddle = createPoint(noHeightVec);
    auto noUnsuitMiddle = createPoint(noUnsuitVec);

    double noLengthFrechet
        = DiscreteFrechet(routes[noLengthMiddle.routeIndex], routes[middle.routeIndex], *g)
              .calculate();
    double noHeightFrechet
        = DiscreteFrechet(routes[noHeightMiddle.routeIndex], routes[middle.routeIndex], *g)
              .calculate();
    double noUnsuitFrechet
        = DiscreteFrechet(routes[noUnsuitMiddle.routeIndex], routes[middle.routeIndex], *g)
              .calculate();

    if (std::max({ noLengthFrechet, noHeightFrechet, noUnsuitFrechet }) < 100) {
      break;
    }

    if (noLengthFrechet > noHeightFrechet) {
      if (noLengthFrechet > noUnsuitFrechet) {
        main = triangles.emplace_back(middle, main.b(), main.c());
      } else {
        main = triangles.emplace_back(main.a(), main.b(), middle);
      }
    } else {
      if (noUnsuitFrechet > noHeightFrechet) {
        main = triangles.emplace_back(main.a(), main.b(), middle);
      } else {
        main = triangles.emplace_back(main.a(), middle, main.c());
      }
    }
  }

  std::cout << "Deciding" << '\n';

  std::optional<Point> maxPoint;
  double maxDistance = -1;

  std::vector<double> distances(routes.size() - 1, -1);

  auto updateDistance = [&](Point p) {
    auto index = p.routeIndex;
    // Exclude the extrem configurations (1/0/0), (0/1/0) and (0/0/1) by excluding the first 3
    // indices
    if (index > 2 && distances[index] < 0) {
      distances[index] = DiscreteFrechet(routes[index], routes[middle.routeIndex], *g).calculate();
      if (distances[index] > maxDistance) {
        maxDistance = distances[index];
        maxPoint = p;
      }
    }
  };

  for (const auto& triangle : triangles) {
    updateDistance(triangle.a());
    updateDistance(triangle.b());
    updateDistance(triangle.c());
  }

  return AlternativeRoutes(
      middle.position, routes[middle.routeIndex], maxPoint->position, routes[maxPoint->routeIndex]);
}

struct TriangleNode {
  Triangle t;
  std::vector<size_t> children;
  TriangleNode(Triangle t)
      : t(std::move(t))
  {
    children.reserve(3);
  }
};

AlternativeRoutes RouteExplorer::collectAndCombine()
{
  routes.clear();
  std::vector<TriangleNode> triangles;
  std::vector<Point> middlePoints;

  auto createChildren = [&](TriangleNode& triangle) -> std::optional<std::pair<Point, Point>> {

    auto middleVec = triangle.t.calculateMiddle();
    auto middle = createPoint(middleVec);

    auto A = triangle.t.a();
    auto B = triangle.t.b();
    auto C = triangle.t.c();

    auto& routeA = routes[A.routeIndex];
    auto& routeB = routes[B.routeIndex];
    auto& routeC = routes[C.routeIndex];
    auto& routeM = routes[middle.routeIndex];

    auto sharedA = calculateSharing(routeA, routeM);
    auto sharedB = calculateSharing(routeB, routeM);
    auto sharedC = calculateSharing(routeC, routeM);

    auto minShared = std::min({ sharedA, sharedB, sharedC });
    std::cout << "smallest sharing value: " << minShared << '\n';
    if (minShared > 0.3) {
      middlePoints.push_back(middle);
      return {};
    }

    triangles.emplace_back(Triangle(middle, B, C));
    triangle.children.push_back(triangles.size() - 1);

    triangles.emplace_back(Triangle(A, middle, C));
    triangle.children.push_back(triangles.size() - 1);

    triangles.emplace_back(Triangle(A, B, middle));
    triangle.children.push_back(triangles.size() - 1);

    if (minShared < 0.1) {
      if (minShared == sharedA) {
        return std::make_pair(A, middle);
      }
      if (minShared == sharedB) {
        return std::make_pair(B, middle);
      }
      if (minShared == sharedC) {
        return std::make_pair(C, middle);
      }
      routes[123443542].edges[1234].costByConfiguration(middleVec);
    }

    return {};
  };

  PosVector lengthVec{ { 1, 0, 0 } };
  PosVector heightVec{ { 0, 1, 0 } };
  PosVector unsuitVec{ { 0, 0, 1 } };

  auto length = createPoint(lengthVec);
  auto height = createPoint(heightVec);
  auto unsuit = createPoint(unsuitVec);

  triangles.emplace_back(Triangle(length, height, unsuit));

  for (size_t i = 0; i < triangles.size(); ++i) {
    auto points = createChildren(triangles[i]);
    if (points && i > 1) {
      auto[point1, point2] = *points;
      return AlternativeRoutes(
          point1.position, routes[point1.routeIndex], point2.position, routes[point2.routeIndex]);
    }
    if (middlePoints.size() > 50) {
      break;
    }
  }

  double bestShared = 1;
  size_t bestI = 0;
  size_t bestJ = 0;

  std::cout << "evaluating triangles" << '\n';
  for (size_t i = 0; i < middlePoints.size(); ++i) {
    auto& routeI = routes[middlePoints[i].routeIndex];
    for (size_t j = i + 1; j < middlePoints.size(); ++j) {
      auto& routeJ = routes[middlePoints[j].routeIndex];
      auto shared = calculateSharing(routeI, routeJ);
      if (shared < bestShared) {
        bestShared = shared;
        bestI = i;
        bestJ = j;
      }
    }
  }

  if (bestJ == 0) {
    return AlternativeRoutes(
        lengthVec, routes[length.routeIndex], heightVec, routes[height.routeIndex]);
  }

  auto& middleI = middlePoints[bestI];
  auto& middleJ = middlePoints[bestJ];

  return AlternativeRoutes(
      middleI.position, routes[middleI.routeIndex], middleJ.position, routes[middleJ.routeIndex]);
}
