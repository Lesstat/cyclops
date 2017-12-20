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
#include <thread>
#include <unordered_map>

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

Route Dijkstra::buildRoute(NodePos node, NodeToEdgeMap previousEdgeS, NodeToEdgeMap previousEdgeT, NodePos from, NodePos to)
{

  Route route{};
  auto curNode = node;
  while (curNode != from) {
    const auto& edgeId = previousEdgeS[curNode];
    const auto& edge = graph->getEdge(edgeId);
    route.costs = route.costs + edge.getCost();
    route.edges.push_front(edge);
    curNode = edge.getSourcePos();
  }

  curNode = node;
  while (curNode != to) {
    const auto& edgeId = previousEdgeT[curNode];
    const auto& edge = graph->getEdge(edgeId);
    route.costs = route.costs + edge.getCost();
    route.edges.push_back(edge);
    curNode = edge.getDestPos();
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

  std::unordered_map<NodePos, EdgeId> previousEdgeS{};

  Queue heapT{ cmp };

  heapT.push(std::make_pair(to, 0));
  touchedT.push_back(to);
  costT[to] = 0;

  std::unordered_map<NodePos, EdgeId> previousEdgeT{};

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
        const auto& e = graph->getEdge(edge);
        NodePos nextNode = e.getDestPos();
        if (graph->getLevelOf(nextNode) >= graph->getLevelOf(node)) {
          double nextCost = cost + e.costByConfiguration(config);
          QueueElem next = std::make_pair(nextNode, nextCost);
          if (nextCost < costS[nextNode]) {
            costS[nextNode] = nextCost;
            touchedS.push_back(nextNode);
            previousEdgeS[nextNode] = e.getId();
            heapS.push(next);
          }
        }
      }
    }

    if (!heapT.empty()) {
      auto[node, cost] = heapT.top();
      heapT.pop();
      if (cost > costT[node]) {
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
        const auto& e = graph->getEdge(edge);
        NodePos nextNode = e.getSourcePos();
        if (graph->getLevelOf(nextNode) >= graph->getLevelOf(node)) {
          double nextCost = cost + e.costByConfiguration(config);
          QueueElem next = std::make_pair(nextNode, nextCost);
          if (nextCost < costT[nextNode]) {
            costT[nextNode] = nextCost;
            touchedT.push_back(nextNode);
            previousEdgeT[nextNode] = e.getId();
            heapT.push(next);
          }
        }
      }
    }
  }
}
