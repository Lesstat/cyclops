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
#include "routeComparator.hpp"
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

    std::pair<size_t, size_t> split()
    {
      if (childEdge1 == 0 || childEdge2 == 0) {
        auto& p1 = tri->points[point1];
        auto& p2 = tri->points[point2];
        PosVector center = (p1.p + p2.p) * 0.5;
        auto pCenter = tri->createPoint(center);
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

public:
    double bestSimilarity = 1;

    Triangle(size_t e1, size_t e2, size_t e3, Triangulation* tri)
        : edge1(e1)
        , edge2(e2)
        , edge3(e3)
        , tri(tri)
    {
      auto pointVec = points();
      noMoreRoutes = true;
      for (size_t i = 0; i < pointVec.size(); ++i) {
        for (size_t j = i + 1; j < pointVec.size(); ++j) {
          auto sim = tri->compare(pointVec[i], pointVec[j]);
          if (sim < tri->threshold) {
            noMoreRoutes = false;
          }
          if (sim < bestSimilarity) {
            bestSimilarity = sim;
          }
        }
      }
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
    bool filled() { return noMoreRoutes; }

    std::vector<size_t> split()
    {
      auto start = std::chrono::high_resolution_clock::now();

      std::vector<size_t> result{};

      if (!noMoreRoutes) {
        result.reserve(4);
        auto edgePair1 = tri->edges[edge1].split();
        auto edgePair2 = tri->edges[edge2].split();
        auto edgePair3 = tri->edges[edge3].split();
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
        result.push_back(tri->createTriangle(halfEdges[0], halfEdges[1], newEdge1));

        auto newEdge2
            = tri->createEdge(tri->edges[halfEdges[2]].point2, tri->edges[halfEdges[3]].point2);
        result.push_back(tri->createTriangle(halfEdges[2], halfEdges[3], newEdge2));

        auto newEdge3
            = tri->createEdge(tri->edges[halfEdges[4]].point2, tri->edges[halfEdges[5]].point2);
        result.push_back(tri->createTriangle(halfEdges[4], halfEdges[5], newEdge3));

        result.push_back(tri->createTriangle(newEdge1, newEdge2, newEdge3));
      }

      auto end = std::chrono::high_resolution_clock::now();
      std::cerr << "splitting the triangle took "
                << std::chrono::duration_cast<ms>(end - start).count() << "ms" << '\n';
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
  std::map<std::pair<size_t, size_t>, double> similarities;

  Dijkstra& d;
  NodePos from;
  NodePos to;
  double threshold;
  std::optional<double> optLength;
  std::optional<double> optHeight;
  std::optional<double> optUnsuitability;
  std::optional<double> maxOpt;

  public:
  Triangulation(Dijkstra& d, NodePos from, NodePos to, double threshold)
      : d(d)
      , from(from)
      , to(to)
      , threshold(threshold)
  {
  }

  void triangulate(size_t maxSplits)
  {
    auto p1 = createPoint(PosVector({ 1, 0, 0 }));
    auto p2 = createPoint(PosVector({ 0, 1, 0 }));
    auto p3 = createPoint(PosVector({ 0, 0, 1 }));

    optLength = points[p1].r.costs.length;
    optHeight = points[p1].r.costs.height;
    optUnsuitability = points[p1].r.costs.unsuitability;
    maxOpt = std::max({ *optLength, *optHeight, *optUnsuitability });

    auto e1 = createEdge(p1, p2);
    auto e2 = createEdge(p1, p3);
    auto e3 = createEdge(p3, p2);

    auto t1 = createTriangle(e1, e2, e3);

    auto simComparator = [this](size_t left, size_t right) {
      return triangles[left].bestSimilarity > triangles[right].bestSimilarity;
    };

    std::priority_queue<size_t, std::vector<size_t>, decltype(simComparator)> q{ simComparator };
    q.push(t1);
    triangles.reserve(maxSplits * 4);

    size_t count = 0;
    while (!q.empty() && count < maxSplits) {
      count++;
      auto tIndex = q.top();
      q.pop();
      auto& t = triangles[tIndex];
      for (auto child : t.split()) {
        if (triangles[child].edgeLength() > 0.04) {
          q.push(child);
        }
      }
    }
  }

  size_t createPoint(const PosVector& p)
  {
    auto start = std::chrono::high_resolution_clock::now();
    Route r;
    if (maxOpt) {
      Config c = p;
      c.length = LengthConfig{ c.length * (*maxOpt / *optLength) };
      c.height = HeightConfig{ c.height * (*maxOpt / *optHeight) };
      c.unsuitability = UnsuitabilityConfig{ c.unsuitability * (*maxOpt / *optUnsuitability) };
      r = *d.findBestRoute(from, to, c);
    } else {
      r = *d.findBestRoute(from, to, p);
    }
    points.emplace_back(p, r, this);

    auto end = std::chrono::high_resolution_clock::now();
    std::cerr << "creating the point/route took "
              << std::chrono::duration_cast<ms>(end - start).count() << "ms" << '\n';
    return points.size() - 1;
  }

  size_t createEdge(size_t p1, size_t p2)
  {
    edges.emplace_back(p1, p2, this);
    return edges.size() - 1;
  }

  size_t createTriangle(size_t e1, size_t e2, size_t e3)
  {
    triangles.emplace_back(e1, e2, e3, this);
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
          return TriTriangle{ points[0], points[1], points[2], t.bestSimilarity == 1.0 };
        });

    return { triPoints, triTriangles };
  }
};

std::tuple<std::vector<TriPoint>, std::vector<TriTriangle>> scaledTriangulation(
    Dijkstra& d, NodePos from, NodePos to, double threshold, size_t maxSplits)
{

  auto start = std::chrono::high_resolution_clock::now();

  Triangulation tri(d, from, to, threshold);
  tri.triangulate(maxSplits);
  auto result = tri.output();

  auto end = std::chrono::high_resolution_clock::now();
  std::cerr << "creating the triangulation took "
            << std::chrono::duration_cast<ms>(end - start).count() << "ms" << '\n';
  return result;
}
