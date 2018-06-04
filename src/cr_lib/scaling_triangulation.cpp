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
#include "loginfo.hpp"
#include "routeComparator.hpp"
#include <future>
#include <iostream>

using ms = std::chrono::milliseconds;

class Triangulation {

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

  struct Edge {
    size_t point1;
    size_t point2;
    Triangulation* tri;

    Edge(size_t point1, size_t point2, Triangulation* tri)
        : point1(point1)
        , point2(point2)
        , tri(tri)
    {
    }

    std::pair<size_t, size_t> split(Dijkstra& d)
    {
      if (childEdge1 == 0 || childEdge2 == 0) {
        auto& p1 = tri->points[point1];
        auto& p2 = tri->points[point2];
        PosVector center = (p1.p + p2.p) * 0.5;
        auto pCenter = tri->createPoint(center, d);
        childEdge1 = tri->createEdge(point1, pCenter);
        childEdge2 = tri->createEdge(point2, pCenter);
      }

      return { childEdge1, childEdge2 };
    }

    double length() const
    {
      auto& p1 = tri->points[point1];
      auto& p2 = tri->points[point2];

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
      auto& p1 = tri->points[pointVec[0]];
      auto& p2 = tri->points[pointVec[1]];
      auto& p3 = tri->points[pointVec[2]];
      auto center = p1.p * 0.33 + p2.p * 0.33 + p3.p * 0.33;

      auto calcRatio = [](double costA, double costB) {
        if (costA < costB) {
          std::swap(costA, costB);
        }
        return costA / costB;
      };

      auto newCost1 = p1.r.costs * center;
      auto newCost2 = p2.r.costs * center;
      auto newCost3 = p3.r.costs * center;

      greatestCostRatio = std::max({ calcRatio(newCost1, newCost2), calcRatio(newCost1, newCost3),
          calcRatio(newCost2, newCost3) });
      noMoreRoutes = greatestCostRatio == 1.0;
    }

    std::vector<size_t> points()
    {
      auto& e1 = tri->edges[edge1];
      auto& e2 = tri->edges[edge2];
      auto& e3 = tri->edges[edge3];

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
            = std::async(std::launch::async, [this]() { return tri->edges[edge1].split(tri->d1); });
        auto edgePair2Future
            = std::async(std::launch::async, [this]() { return tri->edges[edge2].split(tri->d2); });
        auto edgePair3 = tri->edges[edge3].split(tri->d3);
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
            [this](size_t& a, size_t& b) { return tri->edges[a].point1 < tri->edges[b].point1; });

        auto newEdge1
            = tri->createEdge(tri->edges[halfEdges[0]].point2, tri->edges[halfEdges[1]].point2);
        result.push_back(tri->createTriangle(halfEdges[0], halfEdges[1], newEdge1, level + 1));

        auto newEdge2
            = tri->createEdge(tri->edges[halfEdges[2]].point2, tri->edges[halfEdges[3]].point2);
        result.push_back(tri->createTriangle(halfEdges[2], halfEdges[3], newEdge2, level + 1));

        auto newEdge3
            = tri->createEdge(tri->edges[halfEdges[4]].point2, tri->edges[halfEdges[5]].point2);
        result.push_back(tri->createTriangle(halfEdges[4], halfEdges[5], newEdge3, level + 1));

        result.push_back(tri->createTriangle(newEdge1, newEdge2, newEdge3, level + 1));
      }

      return result;
    }
    double edgeLength() const
    {
      auto& e = tri->edges[edge1];
      auto length = e.length();
      return length;
    }
  };

  std::vector<Point> points;
  std::vector<Edge> edges;
  std::vector<Triangle> triangles;

  std::mutex pointMutex;
  std::mutex edgeMutex;

  std::map<std::pair<size_t, size_t>, double> similarities;

  Dijkstra& d1;
  Dijkstra d2;
  Dijkstra d3;
  NodePos from;
  NodePos to;
  std::optional<double> lengthFac;
  std::optional<double> heightFac;
  std::optional<double> unsuitFac;

  public:
  Triangulation(Dijkstra& d, NodePos from, NodePos to)
      : d1(d)
      , d2(d)
      , d3(d)
      , from(from)
      , to(to)
  {
  }

  void triangulate(size_t maxSplits, size_t maxLevel, const bool splitByLevel)
  {
    auto p1Future = std::async(std::launch::async, [this]() {
      return createPoint(PosVector({ 1, 0, 0 }), d1);
    });
    auto p2Future = std::async(std::launch::async, [this]() {
      return createPoint(PosVector({ 0, 1, 0 }), d2);
    });
    auto p3 = createPoint(PosVector({ 0, 0, 1 }), d3);
    auto p1 = p1Future.get();
    auto p2 = p2Future.get();

    double optLength = points[p1].r.costs.length;
    double optHeight = points[p2].r.costs.height;
    double optUnsuitability = points[p3].r.costs.unsuitability;
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

    auto simComparator = [this, splitByLevel](size_t left, size_t right) {
      if (splitByLevel)
        return triangles[left].level > triangles[right].level;
      else
        return triangles[left].greatestCostRatio < triangles[right].greatestCostRatio;
    };

    std::priority_queue<size_t, std::vector<size_t>, decltype(simComparator)> q{ simComparator };
    q.push(t1);
    triangles.reserve(maxSplits * 4);
    points.reserve(maxSplits * 4);
    edges.reserve(3 + maxSplits * 9);

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
  }

  size_t createPoint(const PosVector& p, Dijkstra& d)
  {
    Route r;
    if (lengthFac) {
      Config c = p;
      c.length = LengthConfig{ c.length * *lengthFac };
      c.height = HeightConfig{ c.height * *heightFac };
      c.unsuitability = UnsuitabilityConfig{ c.unsuitability * *unsuitFac };
      r = *d.findBestRoute(from, to, c);
    } else {
      r = *d.findBestRoute(from, to, p);
    }
    std::lock_guard lock(pointMutex);
    points.emplace_back(p, r, this);

    return points.size() - 1;
  }

  size_t createEdge(size_t p1, size_t p2)
  {
    std::lock_guard lock(edgeMutex);
    edges.emplace_back(p1, p2, this);
    return edges.size() - 1;
  }

  size_t createTriangle(size_t e1, size_t e2, size_t e3, size_t level)
  {
    triangles.emplace_back(e1, e2, e3, level, this);
    return triangles.size() - 1;
  }

  double compare(size_t a, size_t b)
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
  std::tuple<std::vector<TriPoint>, std::vector<TriTriangle>> output()
  {
    std::vector<TriPoint> triPoints;
    std::vector<TriTriangle> triTriangles;

    triPoints.reserve(points.size());
    std::transform(points.begin(), points.end(), std::back_inserter(triPoints), [](auto& p) {
      return TriPoint{ p.p, p.r };
    });

    triTriangles.reserve(triangles.size());
    std::transform(
        triangles.begin(), triangles.end(), std::back_inserter(triTriangles), [](auto& t) {
          auto points = t.points();
          return TriTriangle{ points[0], points[1], points[2], t.hasChildren(),
            t.hasNoMoreRoutes() };
        });

    return { triPoints, triTriangles };
  }
};

std::tuple<std::vector<TriPoint>, std::vector<TriTriangle>> scaledTriangulation(Dijkstra& d,
    NodePos from, NodePos to, size_t maxSplits, std::optional<size_t> maxLevel, bool splitByLevel)
{

  Triangulation tri(d, from, to);
  tri.triangulate(maxSplits, maxLevel.value_or(std::numeric_limits<size_t>::max()), splitByLevel);
  auto result = tri.output();

  return result;
}
