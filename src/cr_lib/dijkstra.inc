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
#include "loginfo.hpp"
#include <queue>

const double dmax = std::numeric_limits<double>::max();

template <int Dim> constexpr Cost<Dim> maxCost() { return std::vector<double>(Dim, dmax); };

template <int Dim>
Dijkstra<Dim>::Dijkstra(Graph* g, size_t nodeCount)
    : minCandidate(dmax)
    , costS(nodeCount, dmax)
    , costT(nodeCount, dmax)
    , graph(g)
{
}

template <int Dim> void Dijkstra<Dim>::clearState()
{
  pqPops = 0;
  for (auto nodeId : touchedS) {
    costS[nodeId] = dmax;
  }
  touchedS.clear();
  for (auto nodeId : touchedT) {
    costT[nodeId] = dmax;
  }
  touchedT.clear();
}

template <int Dim> void insertUnpackedEdge(const EdgeId& e, std::deque<EdgeId>& route, bool front)
{
  const auto& edgeA = Edge<Dim>::getEdgeA(e);
  const auto& edgeB = Edge<Dim>::getEdgeB(e);

  if (edgeA) {
    if (front) {
      insertUnpackedEdge<Dim>(*edgeB, route, front);
      insertUnpackedEdge<Dim>(*edgeA, route, front);
    } else {
      insertUnpackedEdge<Dim>(*edgeA, route, front);
      insertUnpackedEdge<Dim>(*edgeB, route, front);
    }
  } else {
    if (front) {
      route.push_front(e);
    } else {
      route.push_back(e);
    }
  }
}

template <int Dim>
Route<Dim> Dijkstra<Dim>::buildRoute(NodePos node, NodeToEdgeMap& previousEdgeS,
    NodeToEdgeMap& previousEdgeT, NodePos from, NodePos to)
{
  using Edge = Edge<Dim>;

  Route route {};
  auto curNode = node;
  while (curNode != from) {
    const auto& edge_id = previousEdgeS[curNode];
    route.costs = route.costs + Edge::getCost(edge_id);
    insertUnpackedEdge<Dim>(edge_id, route.edges, true);
    curNode = Edge::sourcePos(edge_id);
  }

  curNode = node;
  while (curNode != to) {
    const auto& edge_id = previousEdgeT[curNode];
    route.costs = route.costs + Edge::getCost(edge_id);
    insertUnpackedEdge<Dim>(edge_id, route.edges, false);
    curNode = Edge::destPos(edge_id);
  }

  return route;
}

template <int Dim> bool Dijkstra<Dim>::QueueComparator::operator()(QueueElem left, QueueElem right)
{
  auto leftCost = std::get<double>(left);
  auto rightCost = std::get<double>(right);
  return leftCost > rightCost;
}

template <int Dim>
std::optional<Route<Dim>> Dijkstra<Dim>::findBestRoute(NodePos from, NodePos to, Config config)
{
  auto log = Logger::getInstance();

  clearState();
  this->config = config;
  Dijkstra::Queue heap { QueueComparator {} };

  heap.push(std::make_tuple(from, 0, Direction::S));
  touchedS.push_back(from);
  costS[from] = 0;

  NodeToEdgeMap previousEdgeS {};

  heap.push(std::make_tuple(to, 0, Direction::T));
  touchedT.push_back(to);
  costT[to] = 0;

  NodeToEdgeMap previousEdgeT {};

  bool sBigger = false;
  bool tBigger = false;
  minCandidate = dmax;
  std::optional<NodePos> minNode = {};

  while (true) {
    if ((heap.empty()) || (sBigger && tBigger)) {
      *log << "Dijkstra popped " << pqPops << " nodes from PQ"
           << "\n";
      if (minNode.has_value()) {
        return buildRoute(minNode.value(), previousEdgeS, previousEdgeT, from, to);
      }
      return {};
    }

    if (!heap.empty()) {
      auto [node, cost, dir] = heap.top();
      heap.pop();
      pqPops++;
      auto& my_costs = dir == Direction::S ? costS : costT;
      auto& other_costs = dir == Direction::S ? costT : costS;
      auto& bigger = dir == Direction::S ? sBigger : tBigger;
      auto& previous = dir == Direction::S ? previousEdgeS : previousEdgeT;

      if (cost > my_costs[node]) {
        continue;
      }
      if (cost > minCandidate) {
        bigger = true;
        continue;
      }

      if (other_costs[node] != dmax) {
        double candidate = other_costs[node] + my_costs[node];
        if (candidate < minCandidate) {
          minCandidate = candidate;
          minNode = node;
        }
      }
      relaxEdges(node, cost, dir, heap, previous, my_costs);
    }
  }
}

template <int Dim>
void Dijkstra<Dim>::relaxEdges(const NodePos& node, double cost, Direction dir, Queue& heap,
    NodeToEdgeMap& previousEdge, std::vector<double>& costs)
{
  auto& touched = dir == Direction::S ? touchedS : touchedT;
  auto myLevel = graph->getLevelOf(node);
  auto edges
      = dir == Direction::S ? graph->getOutgoingEdgesOf(node) : graph->getIngoingEdgesOf(node);

  std::optional<NodePos> lastNode = {};
  std::optional<double> lastCost = {};
  std::optional<EdgeId> lastEdge = {};
  for (const auto& edge : edges) {
    NodePos nextNode = edge.end;
    if (graph->getLevelOf(nextNode) < myLevel) {
      break;
    }

    if (!lastNode) {
      lastNode = nextNode;
    }
    if (*lastNode != nextNode) {
      if (*lastCost < costs[*lastNode]) {
        costs[*lastNode] = *lastCost;
        touched.push_back(*lastNode);
        previousEdge[*lastNode] = *lastEdge;
        heap.push({ *lastNode, *lastCost, dir });
      }
      lastNode = nextNode;
      lastCost = {};
      lastEdge = {};
    }
    double nextCost = cost + edge.costByConfiguration(config);
    if (!lastCost || *lastCost > nextCost) {
      lastCost = nextCost;
      lastEdge = edge.id;
    }
  }
  if (lastNode && lastCost) {
    if (*lastCost < costs[*lastNode]) {
      costs[*lastNode] = *lastCost;
      touched.push_back(*lastNode);
      previousEdge[*lastNode] = *lastEdge;
      heap.push({ *lastNode, *lastCost, dir });
    }
  }
}

template <int Dim>
bool Dijkstra<Dim>::stallOnDemand(
    const NodePos& node, double cost, Direction dir, std::vector<double>& costs)
{
  auto myLevel = graph->getLevelOf(node);
  const auto& edges
      = dir == Direction::S ? graph->getIngoingEdgesOf(node) : graph->getOutgoingEdgesOf(node);
  for (const auto& edge : edges) {
    if (graph->getLevelOf(edge.end) < myLevel) {
      return false;
    }
    if (costs[edge.end] + edge.cost * config < cost) {
      return true;
    }
  }

  return false;
}

template <int Dim> std::ostream& operator<<(std::ostream& stream, const Config<Dim>& c)
{
  for (size_t i = 0; i < Dim; ++i) {
    if (i > 0) {
      stream << "/";
    }
    stream << c.values[i];
  }
  return stream;
}

template <int Dim> Config<Dim> generateRandomConfig()
{
  std::random_device rd {};
  std::vector<double> conf;
  double current_sum = 0;

  for (size_t i = 0; i < Dim - 1; ++i) {
    std::uniform_real_distribution distribution(0.0, 1.0 - current_sum);
    auto& val = conf.emplace_back(distribution(rd));
    current_sum += val;
  }
  conf.emplace_back(1.0 - current_sum);
  return conf;
}

template <int Dim> void Dijkstra<Dim>::calcScalingFactor(NodePos from, NodePos to, ScalingFactor& f)
{
  double bestValues[Dim];

  for (size_t i = 0; i < Dim; ++i) {
    std::vector conf(Dim, 0.0);
    conf[i] = 1.0;
    auto& costs = findBestRoute(from, to, conf)->costs;
    bestValues[i] = costs.values[i];
  }
  double maxCosts = *std::max_element(&bestValues[0], &bestValues[Dim]);

  for (size_t i = 0; i < Dim; ++i) {
    f[i] = maxCosts / bestValues[i];
  }
  if (std::any_of(&f[0], &f[Dim - 1], [](auto& v) { return std::isnan(v); })) {
    for (auto& v : f) {
      v = 1.0;
    }
  }
}
