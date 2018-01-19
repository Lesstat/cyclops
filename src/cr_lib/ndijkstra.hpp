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

struct RouteWithCount {
  Cost costs;
  size_t pathCount;
  std::deque<Edge> edges;
};

class NormalDijkstra {
  public:
  NormalDijkstra(Graph* g, size_t nodeCount);
  NormalDijkstra(const NormalDijkstra& other) = default;
  NormalDijkstra(NormalDijkstra&& other) noexcept = default;
  virtual ~NormalDijkstra() noexcept = default;
  NormalDijkstra& operator=(const NormalDijkstra& other) = default;
  NormalDijkstra& operator=(NormalDijkstra&& other) noexcept = default;

  std::optional<RouteWithCount> findBestRoute(NodePos from, NodePos to, Config config);

  private:
  void clearState();
  RouteWithCount buildRoute(const NodePos& from, const NodePos& to,
      const std::unordered_map<NodePos, EdgeId>& previousEdge, size_t pathCount);

  using NodeToEdgeMap = std::unordered_map<NodePos, EdgeId>;
  std::vector<double> cost;
  std::vector<NodePos> touched;
  Graph* graph;
};

#endif /* NDIJKSTRA_H */
