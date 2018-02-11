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
    auto& edgeId = previousEdge.at(currentNode).front();
    const auto& edge = Edge::getEdge(edgeId);
    route.costs = route.costs + edge.getCost();
    if (unpack) {
      insertUnpackedEdge(edge, route.edges, true);
    } else {
      route.edges.push_front(edge.getId());
    }
    currentNode = edge.getSourcePos();
  }

  pathCost = route.costs;
  pathCount = route.pathCount;
  return route;
}

RouteWithCount NormalDijkstra::findOtherRoute(const RouteWithCount& route)
{
  if (route.pathCount <= 1) {
    throw std::invalid_argument("no alternative route present");
  }
  std::unordered_set<NodePos> visited;
  auto from = Edge::getEdge(route.edges.front()).getSourcePos();
  auto to = Edge::getEdge(route.edges.back()).getDestPos();
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
void RouteIterator::doubleHeapsize()
{
  if (maxHeapSize > 50000) {
    outputCount = dijkstra->pathCount;
    return;
  }
  maxHeapSize *= 2;
};

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
    for (const auto& edgeId : dijkstra->previousEdge[hTo]) {
      const auto& edge = Edge::getEdge(edgeId);
      const auto& source = edge.getSourcePos();
      RouteWithCount newRoute = hRoute;
      newRoute.costs = newRoute.costs + edge.getCost();
      if (std::abs(dijkstra->cost[source] + edge.costByConfiguration(dijkstra->usedConfig)
              - dijkstra->cost[hTo])
              > 0.00000001
          || newRoute.costs * dijkstra->usedConfig > dijkstra->pathCost * dijkstra->usedConfig) {
        continue;
      }
      newRoute.edges.push_front(edgeId);
      heap.emplace(newRoute, source);
    }
  }
  outputCount = dijkstra->pathCount;
  return {};
}
