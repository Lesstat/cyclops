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

struct RouteWithCount {
  Cost costs;
  size_t pathCount = 1;
  std::deque<EdgeId> edges;
};

class RouteIterator;

class NormalDijkstra {
  public:
  NormalDijkstra(Graph* g, size_t nodeCount, bool unpack = false);
  NormalDijkstra(const NormalDijkstra& other) = default;
  NormalDijkstra(NormalDijkstra&& other) = default;
  virtual ~NormalDijkstra() noexcept = default;
  NormalDijkstra& operator=(const NormalDijkstra& other) = default;
  NormalDijkstra& operator=(NormalDijkstra&& other) = default;

  std::optional<RouteWithCount> findBestRoute(NodePos from, NodePos to, Config config);
  RouteWithCount findOtherRoute(const RouteWithCount& route);
  RouteIterator routeIter(NodePos from, NodePos to);

  friend RouteIterator;

  private:
  void clearState();
  RouteWithCount buildRoute(const NodePos& from, const NodePos& to);

  std::vector<double> cost;
  std::vector<NodePos> touched;
  std::vector<size_t> paths;
  std::unordered_map<NodePos, std::vector<EdgeId>> previousEdge;

  Cost pathCost;
  size_t pathCount;

  Config usedConfig;
  Graph* graph;

  bool unpack;
};

using QueueElem = std::tuple<RouteWithCount, NodePos>;
struct BiggerCost {
  BiggerCost(Config usedConfig)
      : usedConfig(usedConfig)
  {
  }
  bool operator()(QueueElem left, QueueElem right)
  {
    auto leftRoute = std::get<RouteWithCount>(left);
    auto rightRoute = std::get<RouteWithCount>(right);
    return leftRoute.costs * usedConfig > rightRoute.costs * usedConfig;
  }

  Config usedConfig;
};

class RouteIterator {
  public:
  RouteIterator(NormalDijkstra* dijkstra, NodePos from, NodePos to, size_t maxHeapSize = 1000);
  ~RouteIterator() = default;

  std::optional<RouteWithCount> next();
  bool finished();
  void doubleHeapsize();

  private:
  NormalDijkstra* dijkstra;
  size_t maxHeapSize;
  NodePos from;
  NodePos to;
  size_t outputCount = 0;
  using Queue = std::priority_queue<QueueElem, std::vector<QueueElem>, BiggerCost>;
  Queue heap;
};
#endif /* NDIJKSTRA_H */
