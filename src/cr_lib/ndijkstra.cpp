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
#include <set>

NormalDijkstra::NormalDijkstra(Graph* g, size_t nodeCount)
    : cost(nodeCount, std::numeric_limits<double>::max())
    , paths(nodeCount, 0)
    , usedConfig(LengthConfig{ 0.0 }, HeightConfig{ 0.0 }, UnsuitabilityConfig{ 0.0 })
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

  usedConfig = config;
  clearState();
  Queue heap{ cmp };
  heap.push(std::make_tuple(from, 0));
  touched.push_back(from);
  cost[from] = 0;
  paths[from] = 1;

  while (true) {
    if (heap.empty()) {
      return {};
    }
    auto[node, pathCost] = heap.top();
    heap.pop();
    if (node == to) {
      return buildRoute(from, to);
    }
    if (pathCost > cost[node]) {
      continue;
    }

    const auto& outEdges = graph->getOutgoingEdgesOf(node);
    for (const auto& edgeId : outEdges) {
      const auto& edge = Edge::getEdge(edgeId);
      const NodePos& nextNode = edge.getDestPos();
      double nextCost = pathCost + edge.costByConfiguration(config);
      if (nextCost < cost[nextNode]) {
        QueueElem next = std::make_tuple(nextNode, nextCost);
        cost[nextNode] = nextCost;
        paths[nextNode] = paths[node];
        touched.push_back(nextNode);
        previousEdge[nextNode] = { edge.getId() };
        heap.push(std::move(next));
      } else if (std::abs(nextCost - cost[nextNode]) < 0.1) {
        paths[nextNode] += paths[node];
        previousEdge[nextNode].push_back(edge.getId());
      }
    }
  }
}

void NormalDijkstra::clearState()
{
  previousEdge.clear();
  for (const auto& pos : touched) {
    cost[pos] = std::numeric_limits<double>::max();
    paths[pos] = 0;
  }
  touched.clear();
}
void insertUnpackedEdge(const Edge& e, std::deque<Edge>& route, bool front);

RouteWithCount NormalDijkstra::buildRoute(const NodePos& from, const NodePos& to)
{

  RouteWithCount route;
  route.pathCount = paths[to];
  auto currentNode = to;
  while (currentNode != from) {
    auto& edgeId = previousEdge.at(currentNode).front();
    const auto& edge = Edge::getEdge(edgeId);
    route.costs = route.costs + edge.getCost();
    insertUnpackedEdge(edge, route.edges, true);
    currentNode = edge.getSourcePos();
  }

  return route;
}

RouteWithCount NormalDijkstra::findOtherRoute(const RouteWithCount& route)
{
  if (route.pathCount <= 1) {
    throw std::invalid_argument("no alternative route present");
  }
  std::set<NodePos> visited;
  auto from = route.edges.front().getSourcePos();
  auto to = route.edges.back().getDestPos();
  bool once = true;

  RouteWithCount newRoute;
  newRoute.pathCount = route.pathCount;
  auto currentNode = to;
  while (currentNode != from) {
    visited.emplace(currentNode);
    auto& edgeIds = previousEdge.at(currentNode);

    bool found = false;
    for (auto edgeId = edgeIds.begin(); edgeId != edgeIds.end(); ++edgeId) {
      if (once && edgeIds.size() > 1 && edgeId == edgeIds.begin()) {
        once = false;
        continue;
      }
      const auto& edge = Edge::getEdge(*edgeId);
      if (visited.find(edge.getSourcePos()) == visited.end()) {
        newRoute.costs = newRoute.costs + edge.getCost();
        insertUnpackedEdge(edge, newRoute.edges, true);
        currentNode = edge.getSourcePos();
        found = true;
        break;
      }
    }
    if (!found) {
      std::cout << "infinite loop" << '\n';
    }
  }

  return newRoute;
}
