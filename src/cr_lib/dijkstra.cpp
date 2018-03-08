/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2017  Florian Barth

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
#include "dijkstra.hpp"
#include <queue>

const double dmax = std::numeric_limits<double>::max();
const short smax = std::numeric_limits<short>::max();
const Cost maxCost{ Length(dmax), Height(smax), Unsuitability(smax) };

Dijkstra::Dijkstra(Graph* g, size_t nodeCount)
    : costS(nodeCount, dmax)
    , costT(nodeCount, dmax)
    , graph(g)
{
}

void Dijkstra::clearState()
{
  for (auto nodeId : touchedS) {
    costS[nodeId] = dmax;
  }
  touchedS.clear();
  for (auto nodeId : touchedT) {
    costT[nodeId] = dmax;
  }
  touchedT.clear();
}

void insertUnpackedEdge(const Edge& e, std::deque<Edge>& route, bool front)
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
      route.push_front(e);
    } else {
      route.push_back(e);
    }
  }
}

Route Dijkstra::buildRoute(NodePos node, NodeToEdgeMap previousEdgeS, NodeToEdgeMap previousEdgeT,
    NodePos from, NodePos to)
{

  Route route{};
  auto curNode = node;
  while (curNode != from) {
    const auto& edge = previousEdgeS[curNode];
    route.costs = route.costs + edge.cost;
    insertUnpackedEdge(Edge::getEdge(edge.id), route.edges, true);
    curNode = edge.begin;
  }

  curNode = node;
  while (curNode != to) {
    const auto& edge = previousEdgeT[curNode];
    route.costs = route.costs + edge.cost;
    insertUnpackedEdge(Edge::getEdge(edge.id), route.edges, false);
    curNode = edge.begin;
  }

  return route;
}

std::optional<Route> Dijkstra::findBestRoute(NodePos from, NodePos to, Config config)
{
  using QueueElem = std::pair<NodePos, double>;
  auto cmp = [](QueueElem left, QueueElem right) {
    auto leftCost = std::get<double>(left);
    auto rightCost = std::get<double>(right);
    return leftCost > rightCost;
  };
  using Queue = std::priority_queue<QueueElem, std::vector<QueueElem>, decltype(cmp)>;

  clearState();
  Queue heapS{ cmp };

  heapS.push(std::make_pair(from, 0));
  touchedS.push_back(from);
  costS[from] = 0;

  NodeToEdgeMap previousEdgeS{};

  Queue heapT{ cmp };

  heapT.push(std::make_pair(to, 0));
  touchedT.push_back(to);
  costT[to] = 0;

  NodeToEdgeMap previousEdgeT{};

  bool sBigger = false;
  bool tBigger = false;
  double minCandidate = dmax;
  std::optional<NodePos> minNode = {};

  while (true) {
    if (heapS.empty() && heapT.empty()) {
      if (minNode.has_value()) {
        return buildRoute(minNode.value(), previousEdgeS, previousEdgeT, from, to);
      }
      return {};
    }
    if (sBigger && tBigger) {
      return buildRoute(minNode.value(), previousEdgeS, previousEdgeT, from, to);
    }

    if (!heapS.empty()) {
      auto[node, cost] = heapS.top();
      heapS.pop();
      if (cost > costS[node]) {
        continue;
      }
      bool stalled = false;
      const auto& inEdges = graph->getIngoingEdgesOf(node);
      for (const auto& edge : inEdges) {
        if (graph->getLevelOf(edge.end) < graph->getLevelOf(node)) {
          continue;
        }
        if (costS[edge.end] + edge.cost * config < costS[node]) {
          stalled = true;
          break;
        }
      }
      if (stalled) {
        continue;
      }

      if (cost > minCandidate) {
        sBigger = true;
      }
      if (costT[node] != dmax) {
        double candidate = costT[node] + costS[node];
        if (candidate < minCandidate) {
          minCandidate = candidate;
          minNode = node;
        }
      }

      const auto& outEdges = graph->getOutgoingEdgesOf(node);
      for (const auto& edge : outEdges) {
        NodePos nextNode = edge.end;
        if (graph->getLevelOf(nextNode) < graph->getLevelOf(node)) {
          break;
        }
        double nextCost = cost + edge.costByConfiguration(config);
        if (nextCost < costS[nextNode]) {
          QueueElem next = std::make_pair(nextNode, nextCost);
          costS[nextNode] = nextCost;
          touchedS.push_back(nextNode);
          previousEdgeS[nextNode] = edge;
          heapS.push(next);
        }
      }
    }

    if (!heapT.empty()) {
      auto[node, cost] = heapT.top();
      heapT.pop();
      if (cost > costT[node]) {
        continue;
      }

      bool stalled = false;
      const auto& outEdges = graph->getOutgoingEdgesOf(node);
      for (const auto& edge : outEdges) {
        if (graph->getLevelOf(edge.end) < graph->getLevelOf(node)) {
          continue;
        }
        if (costT[edge.end] + edge.cost * config < costT[node]) {
          stalled = true;
          break;
        }
      }
      if (stalled) {
        continue;
      }

      if (cost > minCandidate) {
        tBigger = true;
      }
      if (costS[node] != dmax) {
        double candidate = costS[node] + costT[node];
        if (candidate < minCandidate) {
          minCandidate = candidate;
          minNode = node;
        }
      }

      const auto& inEdges = graph->getIngoingEdgesOf(node);
      for (const auto& edge : inEdges) {
        NodePos nextNode = edge.end;
        if (graph->getLevelOf(nextNode) < graph->getLevelOf(node)) {
          break;
        }
        double nextCost = cost + edge.costByConfiguration(config);
        if (nextCost < costT[nextNode]) {
          QueueElem next = std::make_pair(nextNode, nextCost);
          costT[nextNode] = nextCost;
          touchedT.push_back(nextNode);
          previousEdgeT[nextNode] = edge;
          heapT.push(next);
        }
      }
    }
  }
}
