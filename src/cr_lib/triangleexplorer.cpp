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
#include <queue>
#include <random>

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

  auto& secondRoute = routes.back();

  auto shared = calculateSharing(firstRoute, secondRoute);
  auto frechet = DiscreteFrechet(firstRoute, secondRoute, *g).calculate();

  return AlternativeRoutes(
      first.position, firstRoute, triangles.back().getMiddle(), secondRoute, shared, frechet);
}

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
  auto& firstRoute = routes[middle.routeIndex];
  auto& secondRoute = routes[maxPoint->routeIndex];

  auto shared = calculateSharing(firstRoute, secondRoute);

  return AlternativeRoutes(
      middle.position, firstRoute, maxPoint->position, secondRoute, shared, maxDistance);
}

AlternativeRoutes RouteExplorer::optimizeSharing()
{
  routes.clear();
  std::vector<Triangle> triangles;
  std::vector<Point> middlePoints;

  auto createChildren = [&](Triangle& triangle) -> std::optional<std::tuple<Point, Point, double>> {

    auto middleVec = triangle.calculateMiddle();
    auto middle = createPoint(middleVec);

    auto A = triangle.a();
    auto B = triangle.b();
    auto C = triangle.c();

    auto& routeA = routes[A.routeIndex];
    auto& routeB = routes[B.routeIndex];
    auto& routeC = routes[C.routeIndex];
    auto& routeM = routes[middle.routeIndex];

    auto sharedA = calculateSharing(routeA, routeM);
    auto sharedB = calculateSharing(routeB, routeM);
    auto sharedC = calculateSharing(routeC, routeM);

    auto minShared = std::min({ sharedA, sharedB, sharedC });
    if (minShared > 0.3) {
      middlePoints.push_back(middle);
      return {};
    }

    triangles.emplace_back(middle, B, C);
    triangles.emplace_back(A, middle, C);
    triangles.emplace_back(A, B, middle);
    if (minShared < 0.1) {
      if (minShared == sharedA) {
        return std::make_tuple(A, middle, minShared);
      }
      if (minShared == sharedB) {
        return std::make_tuple(B, middle, minShared);
      }
      if (minShared == sharedC) {
        return std::make_tuple(C, middle, minShared);
      }
    }

    return {};
  };

  PosVector lengthVec{ { 1, 0, 0 } };
  PosVector heightVec{ { 0, 1, 0 } };
  PosVector unsuitVec{ { 0, 0, 1 } };

  auto length = createPoint(lengthVec);
  auto height = createPoint(heightVec);
  auto unsuit = createPoint(unsuitVec);

  triangles.emplace_back(length, height, unsuit);

  for (size_t i = 0; i < triangles.size(); ++i) {
    auto points = createChildren(triangles[i]);
    if (points) {
      auto[point1, point2, shared] = *points;

      auto& first = routes[point1.routeIndex];
      auto& second = routes[point2.routeIndex];

      double frechet = DiscreteFrechet(first, second, *g).calculate();

      return AlternativeRoutes(point1.position, first, point2.position, second, shared, frechet);
    }
    if (middlePoints.size() > 25) {
      break;
    }
  }

  double bestShared = 1;
  size_t bestI = 0;
  size_t bestJ = 0;

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
    auto& lengthRoute = routes[length.routeIndex];
    auto& heightRoute = routes[height.routeIndex];

    auto shared = calculateSharing(lengthRoute, heightRoute);
    auto frechet = DiscreteFrechet(lengthRoute, heightRoute, *g).calculate();

    return AlternativeRoutes(lengthVec, lengthRoute, heightVec, heightRoute, shared, frechet);
  }

  auto& middleI = middlePoints[bestI];
  auto& middleJ = middlePoints[bestJ];

  auto& first = routes[middleI.routeIndex];
  auto& second = routes[middleJ.routeIndex];

  auto frechet = DiscreteFrechet(first, second, *g).calculate();

  return AlternativeRoutes(middleI.position, first, middleJ.position, second, bestShared, frechet);
}

Config generateRandomConfig()
{
  std::random_device rd{};
  std::uniform_real_distribution lenDist(0.0, 1.0);
  LengthConfig l(lenDist(rd));
  std::uniform_real_distribution heightDist(0.0, 1.0 - l.get());
  HeightConfig h(heightDist(rd));
  UnsuitabilityConfig u(1 - l - h);

  return Config{ l, h, u };
}

AlternativeRoutes RouteExplorer::randomAlternatives()
{
  Route route = {};
  Route route2 = {};
  Config conf1{ LengthConfig{ 0 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } };
  Config conf2{ LengthConfig{ 0 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } };
  double shared = 1.0;
  double threshold = 0.1;

  size_t counter = 0;
  while (shared > threshold) {
    if (++counter % 100 == 0) {
      threshold += 0.1;
    }
    conf1 = generateRandomConfig();
    route = d.findBestRoute(from, to, conf1).value();

    conf2 = generateRandomConfig();
    route2 = d.findBestRoute(from, to, conf2).value();
    shared = calculateSharing(route, route2);
  }

  auto frechet = DiscreteFrechet(route, route2, *g).calculate();

  return AlternativeRoutes(conf1, route, conf2, route2, shared, frechet);
}

AlternativeRoutes RouteExplorer::trulyRandomAlternatives()
{
  auto conf1 = generateRandomConfig();
  auto route = d.findBestRoute(from, to, conf1).value();

  auto conf2 = generateRandomConfig();
  auto route2 = d.findBestRoute(from, to, conf2).value();
  auto shared = calculateSharing(route, route2);

  auto frechet = DiscreteFrechet(route, route2, *g).calculate();

  return AlternativeRoutes(conf1, route, conf2, route2, shared, frechet);
}

std::vector<std::tuple<Config, Route>> RouteExplorer::triangleSplitting()
{
  using QueueElem = std::tuple<Triangle, double>;
  auto cmp = [](const QueueElem& a, const QueueElem& b) {
    return std::get<double>(a) > std::get<double>(b);
  };
  using Queue = std::priority_queue<QueueElem, std::vector<QueueElem>, decltype(cmp)>;

  const double SHARING_THRESHOLD = 0.3;

  Queue triangles{ cmp };

  routes.clear();
  std::vector<Point> points;

  PosVector lengthVec{ { 1, 0, 0 } };
  PosVector heightVec{ { 0, 1, 0 } };
  PosVector unsuitVec{ { 0, 0, 1 } };

  auto length = createPoint(lengthVec);
  auto height = createPoint(heightVec);
  auto unsuit = createPoint(unsuitVec);

  points.push_back(length);
  points.push_back(height);
  points.push_back(unsuit);

  triangles.emplace(Triangle{ length, height, unsuit }, 0.0);

  auto isPointOnLine = [](const PosVector& point, const PosVector& a, const PosVector& b) {
    return std::abs(point.distance(a) + point.distance(b) - a.distance(b)) < 0.000001;
  };

  auto createChildren = [&](Triangle& triangle) {

    auto middleVec = triangle.calculateMiddle();
    auto middle = createPoint(middleVec);
    points.push_back(middle);

    auto A = triangle.a();
    auto B = triangle.b();
    auto C = triangle.c();

    auto& routeA = routes[A.routeIndex];
    auto& routeB = routes[B.routeIndex];
    auto& routeC = routes[C.routeIndex];
    auto& routeM = routes[middle.routeIndex];

    auto sharedA = calculateSharing(routeA, routeM);
    auto sharedB = calculateSharing(routeB, routeM);
    auto sharedC = calculateSharing(routeC, routeM);

    auto minShared = std::min({ sharedA, sharedB, sharedC });
    if (minShared > SHARING_THRESHOLD) {
      return;
    }

    Point* outerPoint1 = nullptr;
    Point* outerPoint2 = nullptr;
    Point* innerPoint = nullptr;
    double innerShared = 0;

    auto checkSideSplit = [&](const PosVector& corner1, const PosVector& corner2) {
      if (isPointOnLine(A.position, corner1, corner2)) {
        outerPoint1 = &A;
      } else {
        innerPoint = &A;
        innerShared = sharedA;
      }
      if (isPointOnLine(B.position, corner1, corner2)) {
        if (outerPoint1 == nullptr) {
          outerPoint1 = &B;
        } else {
          outerPoint2 = &B;
        }
      } else {
        if (innerPoint == nullptr) {
          innerPoint = &B;
          innerShared = sharedB;
        } else {
          return false;
        }
      }
      if (isPointOnLine(C.position, corner1, corner2)) {
        if (outerPoint1 == nullptr) {
          return false;
        }
        outerPoint2 = &C;
      } else {
        innerPoint = &C;
        innerShared = sharedC;
      }
      return outerPoint1 != nullptr && outerPoint2 != nullptr && innerPoint != nullptr;
    };

    if ((checkSideSplit(lengthVec, heightVec) || checkSideSplit(lengthVec, unsuitVec)
            || checkSideSplit(heightVec, unsuitVec))
        && (innerShared > SHARING_THRESHOLD)) {
      std::cout << "Alternative split" << '\n';
      auto sideCenterVec = (outerPoint1->position + outerPoint2->position) * 0.5;
      auto sideCenter = createPoint(sideCenterVec);
      points.push_back(sideCenter);

      triangles.emplace(Triangle{ *outerPoint1, sideCenter, *innerPoint }, minShared);
      triangles.emplace(Triangle{ *outerPoint2, sideCenter, *innerPoint }, minShared);
    } else {
      triangles.emplace(Triangle{ middle, B, C }, minShared);
      triangles.emplace(Triangle{ A, middle, C }, minShared);
      triangles.emplace(Triangle{ A, B, middle }, minShared);
    }

  };

  while (!triangles.empty()) {
    auto triangle = triangles.top();
    triangles.pop();
    createChildren(std::get<Triangle>(triangle));
    if (points.size() > 12) {
      std::cout << "reached point limit" << '\n';
      break;
    }
  }
  std::vector<std::tuple<Config, Route>> result;
  result.reserve(routes.size());
  for (const auto& point : points) {
    result.emplace_back(point.position, routes[point.routeIndex]);
  }
  std::cout << "Returning " << result.size() << " routes." << '\n';
  return result;
}
