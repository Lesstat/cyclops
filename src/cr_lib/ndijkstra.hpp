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
#ifndef NDIJKSTRA_H
#define NDIJKSTRA_H
#include "dijkstra.hpp"
#include <queue>
#include <unordered_map>

template <int Dim> struct RouteWithCount {
  Cost<Dim> costs;
  size_t pathCount = 1;
  std::deque<EdgeId> edges;
  RouteWithCount& operator=(const RouteWithCount& rhs) = default;
};

template <int Dim> class RouteIterator;

using QueueElem = std::tuple<NodePos, double>;
struct BiggerPathCost {
  bool operator()(QueueElem left, QueueElem right)
  {
    auto leftCost = std::get<double>(left);
    auto rightCost = std::get<double>(right);
    return leftCost > rightCost;
  }
};
using Queue = std::priority_queue<QueueElem, std::vector<QueueElem>, BiggerPathCost>;

template <int Dim> class NormalDijkstra {
  public:
  using GraphD = Graph<Dim>;
  using ConfigD = Config<Dim>;
  using CostD = Cost<Dim>;
  using EdgeD = Edge<Dim>;
  using HalfEdgeD = HalfEdge<Dim>;
  using RouteWithCountD = RouteWithCount<Dim>;
  using RouteIteratorD = RouteIterator<Dim>;

  NormalDijkstra(GraphD* g, size_t nodeCount, bool unpack = false);
  NormalDijkstra(const NormalDijkstra& other) = default;
  NormalDijkstra(NormalDijkstra&& other) = default;
  virtual ~NormalDijkstra() noexcept = default;
  NormalDijkstra& operator=(const NormalDijkstra& other) = default;
  NormalDijkstra& operator=(NormalDijkstra&& other) = default;

  std::optional<RouteWithCountD> findBestRoute(NodePos from, NodePos to, const ConfigD& config);
  RouteIteratorD routeIter(NodePos from, NodePos to);

  void saveDotGraph(const EdgeId& inId, const EdgeId& outId);

  friend RouteIteratorD;
  size_t pqPops = 0;

  private:
  void clearState();
  RouteWithCountD buildRoute(const NodePos& from, const NodePos& to);

  std::vector<double> cost;
  std::vector<NodePos> touched;
  std::vector<size_t> paths;
  std::vector<std::vector<HalfEdgeD>> previousEdge;

  NodePos from, to;

  CostD pathCost;
  size_t pathCount;

  ConfigD usedConfig;
  GraphD* graph;
  Queue heap;

  bool unpack;
};

template <int Dim> using RouteQueueElem = std::tuple<RouteWithCount<Dim>, NodePos>;
template <int Dim> struct BiggerRouteCost {
  using ConfigD = Config<Dim>;
  using RouteQueueElemD = RouteQueueElem<Dim>;
  using RouteWithCountD = RouteWithCount<Dim>;

  BiggerRouteCost(ConfigD usedConfig)
      : usedConfig(usedConfig)
  {
  }
  bool operator()(RouteQueueElemD left, RouteQueueElemD right)
  {
    auto leftRoute = std::get<RouteWithCountD>(left);
    auto rightRoute = std::get<RouteWithCountD>(right);
    return leftRoute.costs * usedConfig > rightRoute.costs * usedConfig;
  }

  ConfigD usedConfig;
};

template <int Dim> class RouteIterator {
  public:
  using NormalDijkstraD = NormalDijkstra<Dim>;
  using RouteWithCountD = RouteWithCount<Dim>;
  using RouteQueueElemD = RouteQueueElem<Dim>;
  using BiggerRouteCostD = BiggerRouteCost<Dim>;

  RouteIterator(NormalDijkstraD* dijkstra, NodePos from, NodePos to, size_t maxHeapSize = 500);
  ~RouteIterator() = default;

  std::optional<RouteWithCountD> next();
  bool finished();
  void doubleHeapsize();

  private:
  NormalDijkstraD* dijkstra;
  size_t maxHeapSize;
  NodePos from;
  NodePos to;
  size_t outputCount = 0;
  using RouteQueue
      = std::priority_queue<RouteQueueElemD, std::vector<RouteQueueElemD>, BiggerRouteCostD>;
  RouteQueue heap;
};

#include "ndijkstra.inc"
#endif /* NDIJKSTRA_H */
