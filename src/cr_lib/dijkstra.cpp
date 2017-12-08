#include "dijkstra.hpp"
#include <queue>
#include <thread>
#include <unordered_map>

const double max = std::numeric_limits<double>::max();
const Cost maxCost{ Length(max), Height(max), Unsuitability(max) };

Dijkstra::Dijkstra(Graph const& g, size_t nodeCount)
    : costS(nodeCount, max)
    , costT(nodeCount, max)
    , graph(g)
{
}
Dijkstra::Dijkstra(const Dijkstra& other) = default;

Dijkstra::Dijkstra(Dijkstra&& other) noexcept
    : costS(std::move(other.costS))
    , costT(std::move(other.costT))
    , touchedS(std::move(other.touchedS))
    , touchedT(std::move(other.touchedT))
    , graph(std::move(other.graph))
{
}

Dijkstra::~Dijkstra() noexcept = default;

void Dijkstra::clearState()
{
  for (auto nodeId : touchedS) {
    costS[nodeId] = max;
  }
  touchedS.clear();
  for (auto nodeId : touchedT) {
    costT[nodeId] = max;
  }
  touchedT.clear();
}

Route Dijkstra::buildRoute(NodeId node, NodeToEdgeMap previousEdgeS, NodeToEdgeMap previousEdgeT, NodeId from, NodeId to)
{
  Route route{};
  auto curNode = node;
  while (curNode != from) {
    const auto& edgeId = previousEdgeS[curNode];
    const auto& edge = graph.getEdge(edgeId);
    route.costs = route.costs + edge.getCost();
    route.nodes.push_front(graph.getNode(curNode));
    curNode = edge.getSourceId();
  }
  route.nodes.push_front(graph.getNode(curNode));

  curNode = node;
  while (curNode != to) {
    const auto& edgeId = previousEdgeT[curNode];
    const auto& edge = graph.getEdge(edgeId);
    route.costs = route.costs + edge.getCost();
    if (curNode != node) {
      route.nodes.push_back(graph.getNode(curNode));
    }
    curNode = edge.getDestId();
  }
  route.nodes.push_back(graph.getNode(curNode));

  return route;
}

std::optional<Route> Dijkstra::findBestRoute(NodeId from, NodeId to, Config config)
{
  using QueueElem = std::pair<NodeId, double>;
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

  std::unordered_map<NodeId, EdgeId> previousEdgeS{};

  Queue heapT{ cmp };

  heapT.push(std::make_pair(to, 0));
  touchedT.push_back(to);
  costT[to] = 0;

  std::unordered_map<NodeId, EdgeId> previousEdgeT{};

  bool sBigger = false;
  bool tBigger = false;
  double minCandidate = max;
  std::optional<NodeId> minNode = {};

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
      if (costT[node] != max) {
        double candidate = costT[node] + costS[node];
        minCandidate = std::min(candidate, minCandidate);
        minNode = node;
      }

      auto[edge, end] = graph.getOutgoingEdgesOf(node); //NOLINT
      for (; edge != end; ++edge) {
        const auto& e = graph.getEdge(*edge);
        NodeId nextNode = e.getDestId();
        if (graph.getLevelOf(nextNode) >= graph.getLevelOf(node)) {
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
        sBigger = true;
      }
      if (costS[node] != max) {
        double candidate = costS[node] + costT[node];
        minCandidate = std::min(candidate, minCandidate);
        minNode = node;
      }

      auto[edge, end] = graph.getIngoingEdgesOf(node); //NOLINT
      for (; edge != end; ++edge) {
        const auto& e = graph.getEdge(*edge);
        NodeId nextNode = e.getSourceId();
        if (graph.getLevelOf(nextNode) >= graph.getLevelOf(node)) {
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
