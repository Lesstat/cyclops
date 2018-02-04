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
#include <set>

struct RouteWithCount {
  Cost costs;
  size_t pathCount = 1;
  std::deque<EdgeId> edges;
};

class NormalDijkstra {
  public:
  NormalDijkstra(Graph* g, size_t nodeCount, bool unpack = false);
  NormalDijkstra(const NormalDijkstra& other) = default;
  NormalDijkstra(NormalDijkstra&& other) noexcept = default;
  virtual ~NormalDijkstra() noexcept = default;
  NormalDijkstra& operator=(const NormalDijkstra& other) = default;
  NormalDijkstra& operator=(NormalDijkstra&& other) noexcept = default;

  std::optional<RouteWithCount> findBestRoute(NodePos from, NodePos to, Config config);
  RouteWithCount findOtherRoute(const RouteWithCount& route);

  std::vector<RouteWithCount> findAllBestRoutes(
      const NodePos& from, const NodePos& to, const size_t& max);

  private:
  void clearState();
  RouteWithCount buildRoute(const NodePos& from, const NodePos& to);

  void findRoutes(const NodePos& from, const NodePos& to, const size_t& max);

  std::vector<double> cost;
  std::vector<NodePos> touched;
  std::vector<size_t> paths;
  std::unordered_map<NodePos, std::vector<EdgeId>> previousEdge;

  std::vector<RouteWithCount> allRoutes;

  Cost pathCost;
  size_t pathCount;

  Config usedConfig;
  Graph* graph;

  bool unpack;
};

#endif /* NDIJKSTRA_H */
