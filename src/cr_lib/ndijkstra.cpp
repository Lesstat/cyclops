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
#include "ndijkstra.hpp"

NormalDijkstra::NormalDijkstra(Graph* g, size_t nodeCount)
    : cost(nodeCount, std::numeric_limits<double>::max())
    , graph(g)
{
}

std::optional<RouteWithCount> NormalDijkstra::findBestRoute(NodePos from, NodePos to, Config config)
{

  using QueueElem = std::tuple<NodePos, double>;
  auto cmp = [](QueueElem left, QueueElem right) {
    auto leftCost = std::get<double>(left);
    auto rightCost = std::get<double>(right);
    return leftCost > rightCost;
  };
  using Queue = std::priority_queue<QueueElem, std::vector<QueueElem>, decltype(cmp)>;

  clearState();
  Queue heap{ cmp };
  heap.push(std::make_tuple(from, 0));
  touched.push_back(from);
  cost[from] = 0;
  std::unordered_map<NodePos, EdgeId> previousEdge{};

  while (true) {
    if (heap.empty()) {
      return {};
    }
    auto[node, pathCost] = heap.top();
    heap.pop();
    if (node == to) {
      return buildRoute(from, to, previousEdge);
    }
    if (pathCost > cost[node]) {
      continue;
    }

    const auto& outEdges = graph->getOutgoingEdgesOf(node);
    for (const auto& edge : outEdges) {
      const NodePos& nextNode = edge.end;
      double nextCost = pathCost + edge.costByConfiguration(config);
      if (nextCost < cost[nextNode]) {
        QueueElem next = std::make_tuple(nextNode, nextCost);
        cost[nextNode] = nextCost;
        touched.push_back(nextNode);
        previousEdge[nextNode] = edge.id;
        heap.push(std::move(next));
      }
    }
  }
}

void NormalDijkstra::clearState()
{
  for (const auto& pos : touched) {
    cost[pos] = std::numeric_limits<double>::max();
  }
  touched.clear();
}
void insertUnpackedEdge(const Edge& e, std::deque<Edge>& route, bool front);

RouteWithCount NormalDijkstra::buildRoute(
    const NodePos& from, const NodePos& to, const std::unordered_map<NodePos, EdgeId>& previousEdge)
{
  RouteWithCount route;
  auto currentNode = to;
  while (currentNode != from) {
    auto& edgeId = previousEdge.at(currentNode);
    const auto& edge = Edge::getEdge(edgeId);
    route.costs = route.costs + edge.getCost();
    insertUnpackedEdge(edge, route.edges, true);
    currentNode = edge.getSourcePos();
  }

  return route;
}
