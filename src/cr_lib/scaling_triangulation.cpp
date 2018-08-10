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

#include "scaling_triangulation.hpp"
#include "ilp_independent_set.hpp"
#include "loginfo.hpp"
#include "routeComparator.hpp"
#include <future>
#include <iostream>
#include <map>

using ms = std::chrono::milliseconds;
namespace c = std::chrono;

struct Point;
class Triangle;
struct TriEdge;
class Triangulation;

class TrianglePrio {

  protected:
  Triangulation* tri;

  public:
  explicit TrianglePrio(Triangulation* tri)
      : tri(tri)
  {
  }
  virtual double prio(size_t level, size_t p1, size_t p2, size_t p3) = 0;
  virtual bool noMoreRoutes(size_t p1, size_t p2, size_t p3) = 0;
  virtual bool comparator(size_t left, size_t right) const = 0;
};

class Triangulation {

  std::vector<Point> points;
  std::vector<TriEdge> edges;
  std::vector<Triangle> triangles;

  std::mutex pointMutex;
  std::mutex edgeMutex;

  std::map<std::pair<size_t, size_t>, double> similarities;

  NodePos from;
  NodePos to;
  std::optional<double> lengthFac;
  std::optional<double> heightFac;
  std::optional<double> unsuitFac;
  size_t explore_time = 0;
  size_t recommendation_time = 0;

  public:
  TrianglePrio* prioCalculator;
  Dijkstra& d1;
  Dijkstra d2;
  Dijkstra d3;
  Triangulation(Dijkstra& d, NodePos from, NodePos to)
      : from(from)
      , to(to)
      , d1(d)
      , d2(d)
      , d3(d)
  {
  }

  Point& point(size_t index) { return points[index]; }
  TriEdge& edge(size_t index) { return edges[index]; }
  Triangle& triangle(size_t index) { return triangles[index]; }
  void prioCalculation(TrianglePrio* prio) { prioCalculator = prio; }

  void triangulate(size_t maxSplits, size_t maxLevel);
  size_t createPoint(const PosVector& p, Dijkstra& d);
  size_t createEdge(size_t p1, size_t p2);
  size_t createTriangle(size_t e1, size_t e2, size_t e3, size_t level);
  double compare(size_t a, size_t b);
  TriangulationResult output(double maxOverlap);
};

struct Point {
  PosVector p;
  Route r;
  Triangulation* tri;
  Point(PosVector pos, Route route, Triangulation* tri)
      : p(std::move(pos))
      , r(std::move(route))
      , tri(tri)
  {
  }
};

struct TriEdge {
  size_t point1;
  size_t point2;
  Triangulation* tri;

  TriEdge(size_t point1, size_t point2, Triangulation* tri)
      : point1(point1)
      , point2(point2)
      , tri(tri)
  {
  }

  std::pair<size_t, size_t> split(Dijkstra& d)
  {
    if (childEdge1 == 0 || childEdge2 == 0) {
      auto& p1 = tri->point(point1);
      auto& p2 = tri->point(point2);
      PosVector center = (p1.p + p2.p) * 0.5;
      auto pCenter = tri->createPoint(center, d);
      childEdge1 = tri->createEdge(point1, pCenter);
      childEdge2 = tri->createEdge(point2, pCenter);
    }

    return { childEdge1, childEdge2 };
  }

  double length() const
  {
    auto& p1 = tri->point(point1);
    auto& p2 = tri->point(point2);

    return p1.p.distance(p2.p);
  }

  private:
  size_t childEdge1 = 0;
  size_t childEdge2 = 0;
};

class Triangle {
  size_t edge1;
  size_t edge2;
  size_t edge3;
  Triangulation* tri;
  bool noMoreRoutes = false;
  bool noChildren = true;

  public:
  double greatestCostRatio = 1;
  size_t level;

  Triangle(size_t e1, size_t e2, size_t e3, size_t level, Triangulation* tri)
      : edge1(e1)
      , edge2(e2)
      , edge3(e3)
      , tri(tri)
      , level(level)
  {
    auto pointVec = points();
    greatestCostRatio = tri->prioCalculator->prio(level, pointVec[0], pointVec[1], pointVec[2]);
    noMoreRoutes = tri->prioCalculator->noMoreRoutes(pointVec[0], pointVec[1], pointVec[2]);
  }

  std::vector<size_t> points()
  {
    auto& e1 = tri->edge(edge1);
    auto& e2 = tri->edge(edge2);
    auto& e3 = tri->edge(edge3);

    std::vector<size_t> points{ { e1.point1, e2.point1, e3.point1, e1.point2, e2.point2,
        e3.point2 } };
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    return points;
  }
  bool hasChildren() { return !noChildren; }
  bool hasNoMoreRoutes() { return noMoreRoutes; }

  std::vector<size_t> split()
  {
    std::vector<size_t> result{};

    if (!noMoreRoutes) {
      noChildren = false;
      result.reserve(4);

      auto edgePair1Future
          = std::async(std::launch::async, [this]() { return tri->edge(edge1).split(tri->d1); });
      auto edgePair2Future
          = std::async(std::launch::async, [this]() { return tri->edge(edge2).split(tri->d2); });
      auto edgePair3 = tri->edge(edge3).split(tri->d3);
      auto edgePair1 = edgePair1Future.get();
      auto edgePair2 = edgePair2Future.get();

      std::vector<size_t> halfEdges;
      halfEdges.push_back(edgePair1.first);
      halfEdges.push_back(edgePair1.second);
      halfEdges.push_back(edgePair2.first);
      halfEdges.push_back(edgePair2.second);
      halfEdges.push_back(edgePair3.first);
      halfEdges.push_back(edgePair3.second);
      std::sort(halfEdges.begin(), halfEdges.end(),
          [this](size_t& a, size_t& b) { return tri->edge(a).point1 < tri->edge(b).point1; });

      auto newEdge1
          = tri->createEdge(tri->edge(halfEdges[0]).point2, tri->edge(halfEdges[1]).point2);
      result.push_back(tri->createTriangle(halfEdges[0], halfEdges[1], newEdge1, level + 1));

      auto newEdge2
          = tri->createEdge(tri->edge(halfEdges[2]).point2, tri->edge(halfEdges[3]).point2);
      result.push_back(tri->createTriangle(halfEdges[2], halfEdges[3], newEdge2, level + 1));

      auto newEdge3
          = tri->createEdge(tri->edge(halfEdges[4]).point2, tri->edge(halfEdges[5]).point2);
      result.push_back(tri->createTriangle(halfEdges[4], halfEdges[5], newEdge3, level + 1));

      result.push_back(tri->createTriangle(newEdge1, newEdge2, newEdge3, level + 1));
    }

    return result;
  }
  double edgeLength() const
  {
    auto& e = tri->edge(edge1);
    auto length = e.length();
    return length;
  }
};

void Triangulation::triangulate(size_t maxSplits, size_t maxLevel)
{
  auto start = c::high_resolution_clock::now();
  auto p1Future = std::async(std::launch::async, [this]() {
    return createPoint(PosVector({ 1, 0, 0 }), d1);
  });
  auto p2Future = std::async(std::launch::async, [this]() {
    return createPoint(PosVector({ 0, 1, 0 }), d2);
  });
  auto p3 = createPoint(PosVector({ 0, 0, 1 }), d3);
  auto p1 = p1Future.get();
  auto p2 = p2Future.get();

  double optLength = points[p1].r.costs.values[0];
  double optHeight = points[p2].r.costs.values[1];
  double optUnsuitability = points[p3].r.costs.values[2];
  auto maxOpt = std::max({ optLength, optHeight, optUnsuitability });

  lengthFac = maxOpt / optLength;
  heightFac = maxOpt / optHeight;
  unsuitFac = maxOpt / optUnsuitability;
  auto log = Logger::getInstance();

  *log << "length correction factor:" << *lengthFac << "\\n";
  *log << "height correction factor:" << *heightFac << "\\n";
  *log << "unsuitability correction factor:" << *unsuitFac << "\\n";

  auto e1 = createEdge(p1, p2);
  auto e2 = createEdge(p1, p3);
  auto e3 = createEdge(p3, p2);

  auto t1 = createTriangle(e1, e2, e3, 1);

  auto const comparator
      = [this](size_t left, size_t right) { return prioCalculator->comparator(left, right); };
  std::priority_queue<size_t, std::vector<size_t>, decltype(comparator)> q{ comparator };
  q.push(t1);
  size_t reserveMultiplier = std::max({ size_t(2), maxSplits });
  triangles.reserve(reserveMultiplier * 4);
  points.reserve(reserveMultiplier * 4);
  edges.reserve(3 + reserveMultiplier * 9);

  size_t count = 0;
  while (!q.empty() && count < maxSplits) {
    count++;
    auto tIndex = q.top();
    q.pop();
    auto& t = triangles[tIndex];
    if (t.level > maxLevel) {
      continue;
    }
    for (auto child : t.split()) {
      q.push(child);
    }
  }
  auto end = c::high_resolution_clock::now();
  explore_time = c::duration_cast<ms>(end - start).count();
}

size_t Triangulation::createPoint(const PosVector& p, Dijkstra& d)
{
  Route r;
  if (lengthFac) {
    Config c = p;
    c.values[0] *= *lengthFac;
    c.values[1] *= *heightFac;
    c.values[2] *= *unsuitFac;
    r = *d.findBestRoute(from, to, c);
  } else {
    r = *d.findBestRoute(from, to, p);
  }
  std::lock_guard lock(pointMutex);
  points.emplace_back(p, r, this);

  return points.size() - 1;
}

size_t Triangulation::createEdge(size_t p1, size_t p2)
{
  std::lock_guard lock(edgeMutex);
  edges.emplace_back(p1, p2, this);
  return edges.size() - 1;
}

size_t Triangulation::createTriangle(size_t e1, size_t e2, size_t e3, size_t level)
{
  triangles.emplace_back(e1, e2, e3, level, this);
  return triangles.size() - 1;
}

double Triangulation::compare(size_t a, size_t b)
{
  if (a > b) {
    std::swap(a, b);
  }
  std::pair mapIndex = { a, b };
  if (similarities.count(mapIndex) > 0) {
    return similarities[mapIndex];
  }

  auto& pointA = points[a];
  auto& pointB = points[b];

  auto sharing = calculateSharing(pointA.r, pointB.r);
  similarities[mapIndex] = sharing;
  return sharing;
}
TriangulationResult Triangulation::output(double maxOverlap)
{
  std::vector<TriPoint> triPoints;
  std::vector<TriTriangle> triTriangles;

  std::vector<std::pair<size_t, size_t>> edges;

  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = i + 1; j < points.size(); ++j) {
      if (compare(i, j) >= maxOverlap) {
        edges.emplace_back(i, j);
      }
    }
  }
  auto start = c::high_resolution_clock::now();
  auto independentSet = find_independent_set(points.size(), edges);
  auto end = c::high_resolution_clock::now();
  recommendation_time = c::duration_cast<ms>(end - start).count();

  triPoints.reserve(points.size());

  std::transform(points.begin(), points.end(), std::back_inserter(triPoints), [](auto& p) {
    return TriPoint{ p.p, p.r, false };
  });

  for (auto& n : independentSet) {
    triPoints[n].selected = true;
  }

  triTriangles.reserve(triangles.size());
  std::transform(triangles.begin(), triangles.end(), std::back_inserter(triTriangles), [](auto& t) {
    auto points = t.points();
    return TriTriangle{ points[0], points[1], points[2], !t.hasChildren(), t.hasNoMoreRoutes() };
  });

  return { triPoints, triTriangles, explore_time, recommendation_time };
}

class CenterAlphaPrio : public TrianglePrio {
  public:
  CenterAlphaPrio(Triangulation* tri)
      : TrianglePrio(tri)
  {
  }
  double prio(size_t, size_t p1, size_t p2, size_t p3) override
  {
    auto& point_p1 = tri->point(p1);
    auto& point_p2 = tri->point(p2);
    auto& point_p3 = tri->point(p3);

    auto center = point_p1.p * 0.33 + point_p2.p * 0.33 + point_p3.p * 0.33;

    auto calcRatio = [](double costA, double costB) {
      if (costA < costB) {
        std::swap(costA, costB);
      }
      return costA / costB;
    };

    auto newCost1 = point_p1.r.costs * center;
    auto newCost2 = point_p2.r.costs * center;
    auto newCost3 = point_p3.r.costs * center;

    return std::max({ calcRatio(newCost1, newCost2), calcRatio(newCost1, newCost3),
        calcRatio(newCost2, newCost3) });
  };
  bool noMoreRoutes(size_t p1, size_t p2, size_t p3) override
  {
    return tri->compare(p1, p2) == 1 && tri->compare(p2, p3) == 1;
  };

  bool comparator(size_t left, size_t right) const override
  {
    return tri->triangle(left).greatestCostRatio < tri->triangle(right).greatestCostRatio;
  };
};

class LevelPrio : public CenterAlphaPrio {
  public:
  LevelPrio(Triangulation* tri)
      : CenterAlphaPrio(tri)
  {
  }
  double prio(size_t level, size_t, size_t, size_t) override { return level; }
  bool comparator(size_t left, size_t right) const override
  {
    return tri->triangle(left).greatestCostRatio > tri->triangle(right).greatestCostRatio;
  };
};

class SimRouteCountPrio : public TrianglePrio {
  double maxOverlap;

  public:
  SimRouteCountPrio(Triangulation* tri, double maxOverlap)
      : TrianglePrio(tri)
      , maxOverlap(maxOverlap){};

  bool noMoreRoutes(size_t p1, size_t p2, size_t p3) override
  {
    return tri->compare(p1, p2) == 1 && tri->compare(p2, p3) == 1;
  };

  double prio(size_t, size_t p1, size_t p2, size_t p3) override
  {

    auto result = 0;
    auto lastPoint = std::max({ p1, p2, p3 });
    for (size_t i = 0; i < lastPoint; ++i) {
      if (i != p1 && tri->compare(i, p1) > maxOverlap) {
        result++;
      }
      if (i != p2 && tri->compare(i, p2) > maxOverlap) {
        result++;
      }
      if (i != p3 && tri->compare(i, p3) > maxOverlap) {
        result++;
      }
    }
    return result;
  }
  bool comparator(size_t left, size_t right) const override
  {
    return tri->triangle(left).greatestCostRatio > tri->triangle(right).greatestCostRatio;
  }
};

TriangulationResult scaledTriangulation(Dijkstra& d, NodePos from, NodePos to, size_t maxSplits,
    std::optional<size_t> maxLevel, bool splitByLevel, double maxOverlap)
{

  Triangulation tri(d, from, to);
  CenterAlphaPrio centerPrio(&tri);
  LevelPrio levelPrio(&tri);
  SimRouteCountPrio simPrio(&tri, maxOverlap);
  if (splitByLevel) {
    tri.prioCalculation(&levelPrio);
  } else {
    tri.prioCalculation(&simPrio);
  }
  tri.triangulate(maxSplits, maxLevel.value_or(std::numeric_limits<size_t>::max()));
  auto result = tri.output(maxOverlap);

  return result;
}
