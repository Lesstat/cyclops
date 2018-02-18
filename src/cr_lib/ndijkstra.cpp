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
#include <unordered_set>

NormalDijkstra::NormalDijkstra(Graph* g, size_t nodeCount, bool unpack)
    : cost(nodeCount, std::numeric_limits<double>::max())
    , paths(nodeCount, 0)
    , previousEdge(nodeCount)
    , pathCount(0)
    , usedConfig(LengthConfig{ 0.0 }, HeightConfig{ 0.0 }, UnsuitabilityConfig{ 0.0 })
    , graph(g)
    , unpack(unpack)
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
    for (const auto& edge : outEdges) {
      const NodePos& nextNode = edge.end;
      double nextCost = pathCost + edge.costByConfiguration(config);
      if (nextCost < cost[nextNode]) {
        QueueElem next = std::make_tuple(nextNode, nextCost);
        cost[nextNode] = nextCost;
        paths[nextNode] = paths[node];
        touched.push_back(nextNode);
        previousEdge[nextNode] = { edge };
        heap.push(std::move(next));
      } else if (std::abs(nextCost - cost[nextNode]) < 0.1) {
        paths[nextNode] += paths[node];
        previousEdge[nextNode].push_back(edge);
      }
    }
  }
}

void NormalDijkstra::clearState()
{
  for (const auto& pos : touched) {
    cost[pos] = std::numeric_limits<double>::max();
    paths[pos] = 0;
    previousEdge[pos].clear();
  }
  touched.clear();
  pathCost = Cost{};
  pathCount = 0;
}
void insertUnpackedEdge(const Edge& e, std::deque<EdgeId>& route, bool front)
{
  const auto& edgeA = e.getEdgeA();
  const auto& edgeB = e.getEdgeB();

  if (edgeA) {
    if (front) {
      insertUnpackedEdge(Edge::getEdge(*edgeB), route, front);
      insertUnpackedEdge(Edge::getEdge(*edgeA), route, front);
    } else {

      insertUnpackedEdge(Edge::getEdge(*edgeA), route, front);
      insertUnpackedEdge(Edge::getEdge(*edgeB), route, front);
    }
  } else {
    if (front) {
      route.push_front(e.getId());
    } else {
      route.push_back(e.getId());
    }
  }
}

RouteWithCount NormalDijkstra::buildRoute(const NodePos& from, const NodePos& to)
{

  RouteWithCount route;
  route.pathCount = paths[to];
  auto currentNode = to;
  while (currentNode != from) {
    auto& edge = previousEdge.at(currentNode).front();
    route.costs = route.costs + edge.cost;
    if (unpack) {
      insertUnpackedEdge(Edge::getEdge(edge.id), route.edges, true);
    } else {
      route.edges.push_front(edge.id);
    }
    currentNode = edge.begin;
  }

  pathCost = route.costs;
  pathCount = route.pathCount;
  return route;
}

RouteIterator NormalDijkstra::routeIter(NodePos from, NodePos to)
{
  return RouteIterator(this, from, to);
}

RouteIterator::RouteIterator(NormalDijkstra* dijkstra, NodePos from, NodePos to, size_t maxHeapSize)
    : dijkstra(dijkstra)
    , maxHeapSize(maxHeapSize)
    , from(from)
    , to(to)
    , heap(BiggerCost(dijkstra->usedConfig))
{
  heap.emplace(RouteWithCount(), to);
}

bool RouteIterator::finished() { return outputCount >= dijkstra->pathCount; }
void RouteIterator::doubleHeapsize() { maxHeapSize *= 2; };

std::optional<RouteWithCount> RouteIterator::next()
{

  while (!heap.empty()) {
    if (finished()) {
      std::cout << "finished" << '\n';
      return {};
    }
    if (heap.size() > maxHeapSize) {
      return {};
    }

    auto[hRoute, hTo] = heap.top();
    heap.pop();

    if (hTo == from) {
      outputCount++;
      return hRoute;
    }
    for (const auto& edge : dijkstra->previousEdge[hTo]) {
      const auto& source = edge.begin;
      RouteWithCount newRoute = hRoute;
      newRoute.costs = newRoute.costs + edge.cost;
      if (std::abs(dijkstra->cost[source] + edge.costByConfiguration(dijkstra->usedConfig)
              - dijkstra->cost[hTo])
              > 0.00000001
          || newRoute.costs * dijkstra->usedConfig > dijkstra->pathCost * dijkstra->usedConfig) {
        continue;
      }
      newRoute.edges.push_front(edge.id);
      heap.emplace(newRoute, source);
    }
  }
  outputCount = dijkstra->pathCount;
  return {};
}
