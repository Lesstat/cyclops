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
#include "contractor.hpp"
#include "linearProgram.hpp"
#include "multiqueue.hpp"
#include "ndijkstra.hpp"
#include <any>
#include <chrono>
#include <iostream>
#include <memory>

class StatisticsCollector {
  public:
  enum class CountType { shortestPath, repeatingConfig, toManyConstraints };

  StatisticsCollector(bool active)
      : active(active){};
  StatisticsCollector(const StatisticsCollector& other) = default;
  StatisticsCollector(StatisticsCollector&& other) noexcept = default;
  virtual ~StatisticsCollector() noexcept
  {
    if (!active || shortCount == 0) {
      return;
    }
    std::lock_guard guard(key);
    std::cout << shortCount << "\t\t" << sameCount << "\t\t\t" << toManyConstraints << "\t\t\t"
              << lpMax << "\t\t" << constMax << '\n';
  }
  StatisticsCollector& operator=(const StatisticsCollector& other) = default;
  StatisticsCollector& operator=(StatisticsCollector&& other) noexcept = default;

  static void printHeader()
  {
    std::cout << "| \t\t Reasons for shortcut creation \t\t | \t\t  Max values \t\t|  " << '\n';
    std::cout << "short \t\t repeating \t\t constraints \t\t lp calls \t max constraints" << '\n';
  }

  void countShortcut(CountType t)
  {
    switch (t) {
    case CountType::shortestPath: {
      ++shortCount;
      break;
    }
    case CountType::repeatingConfig: {
      ++sameCount;
      break;
    }
    case CountType::toManyConstraints: {
      ++toManyConstraints;
      break;
    }
    }
  }
  void recordMaxValues(size_t lpCalls, size_t constraints)
  {
    lpMax = std::max(lpCalls, lpMax);
    constMax = std::max(constraints, constMax);
  }

  protected:
  private:
  bool active;
  size_t shortCount = 0;
  size_t sameCount = 0;
  size_t toManyConstraints = 0;
  size_t lpMax = 0;
  size_t constMax = 0;
  static std::mutex key;
};
std::mutex StatisticsCollector::key{};

std::pair<bool, std::optional<RouteWithCount>> checkShortestPath(
    NormalDijkstra& d, const HalfEdge& startEdge, const HalfEdge& destEdge, const Config& conf)
{
  auto foundRoute = d.findBestRoute(startEdge.end, destEdge.end, conf);
  if (!foundRoute) {
    return std::make_pair(false, foundRoute);
  }

  auto route = foundRoute.value();
  bool isShortest
      = route.edges.size() == 2 && route.edges[0] == startEdge.id && route.edges[1] == destEdge.id;
  return std::make_pair(isShortest, foundRoute);
}

class ContractingThread {
  MultiQueue& queue;
  Graph& graph;
  StatisticsCollector stats;
  Config config{ LengthConfig{ 0.33 }, HeightConfig{ 0.33 }, UnsuitabilityConfig{ 0.33 } };
  LinearProgram lp;
  HalfEdge in;
  HalfEdge out;
  size_t lpCount = 0;
  NormalDijkstra d;
  std::vector<Edge> shortcuts;
  Cost shortcutCost;
  Cost currentCost;
  std::vector<Cost> constraints;

  public:
  ContractingThread(MultiQueue& queue, Graph& g, bool printStatistics)
      : queue(queue)
      , graph(g)
      , stats(printStatistics)
      , lp(3)
      , d(g.createNormalDijkstra())
  {
    shortcuts.reserve(graph.getNodeCount());
  }

  bool isDominated(const Cost& costs)
  {
    bool dominated = true;
    for (size_t i = 0; i <= Cost::dim; i++) {
      if (costs.values[i] > shortcutCost.values[i]) {
        dominated = false;
        break;
      }
    }
    if (dominated)
      return true;
    return false;
  }

  void addConstraint(const Cost& costs)
  {
    Cost newCost = shortcutCost - costs;
    lp.addConstraint(newCost.values, 0.0);
  }

  void extractRoutesAndAddConstraints(RouteIterator& routes)
  {
    for (auto optRoute = routes.next(); optRoute; optRoute = routes.next()) {
      auto route = *optRoute;
      if (route.edges.size() == 2 && route.edges[0] == in.id && route.edges[1] == out.id) {
        continue;
      }
      addConstraint(route.costs);
      optRoute = routes.next();
    }
  }

  void storeShortcut(StatisticsCollector::CountType type)
  {
    stats.countShortcut(type);
    stats.recordMaxValues(lpCount, lp.constraintCount());
    shortcuts.push_back(Contractor::createShortcut(Edge::getEdge(in.id), Edge::getEdge(out.id)));
  };

  bool testConfig(const Config& c)
  {
    auto [isShortest, foundRoute] = checkShortestPath(d, in, out, c);

    if (!foundRoute || foundRoute->edges.empty()) {
      stats.recordMaxValues(lpCount, lp.constraintCount());
      return true;
    }

    if ((isShortest && foundRoute->pathCount == 1)) {
      storeShortcut(StatisticsCollector::CountType::shortestPath);
      return true;
    }
    currentCost = foundRoute->costs;
    constraints.push_back(currentCost);

    if (isDominated(currentCost)) {
      return true;
    }
    return false;
  }

  std::vector<Edge> operator()()
  {
    std::vector<std::any> messages;
    while (true) {
      messages.clear();
      if (queue.receive_some(messages, 20) == 0 && queue.closed()) {
        return shortcuts;
      }
      for (auto& msg : messages) {
        auto pair = std::any_cast<EdgePair>(msg);
        bool warm = false;
        if (pair.in.end == in.end && pair.out.end == out.end) {
          warm = true;
        } else {
          constraints.clear();
        }

        in = pair.in;
        out = pair.out;

        config = Config{ LengthConfig{ 0.33 }, HeightConfig{ 0.33 }, UnsuitabilityConfig{ 0.33 } };
        shortcutCost = in.cost + out.cost;

        if (!warm) {
          warm = true;
          if (testConfig(Config{ LengthConfig{ 1 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } })
              || testConfig(
                     Config{ LengthConfig{ 0 }, HeightConfig{ 1 }, UnsuitabilityConfig{ 0 } })
              || testConfig(
                     Config{ LengthConfig{ 0 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 1 } })) {
            continue;
          }
        }

        for (auto c : constraints) {
          if (isDominated(c))
            continue;
        }

        lpCount = 0;
        lp = LinearProgram{ 3 };
        lp.objective({ 1.0, 1.0, 1.0 });
        lp.addConstraint({ 1.0, 1.0, 1.0 }, 1.0, 1.0);
        while (true) {

          for (auto& c : constraints) {
            addConstraint(c);
          }
          if (testConfig(config)) {
            break;
          }
          if (lpCount > 10) {
            storeShortcut(StatisticsCollector::CountType::toManyConstraints);
            break;
          }

          ++lpCount;
          if (!lp.solve()) {
            stats.recordMaxValues(lpCount, lp.constraintCount());
            break;
          }
          auto values = lp.variableValues();

          Config newConfig{ LengthConfig{ values[0] }, HeightConfig{ values[1] },
            UnsuitabilityConfig{ values[2] } };
          config = newConfig;
        }
      }
    }
  }
};

Contractor::Contractor(bool printStatistics)
    : printStatistics(printStatistics)
{
}

std::pair<bool, std::optional<RouteWithCount>> Contractor::isShortestPath(
    NormalDijkstra& d, const HalfEdge& startEdge, const HalfEdge& destEdge, const Config& conf)
{
  return checkShortestPath(d, startEdge, destEdge, conf);
}

Edge Contractor::createShortcut(const Edge& e1, const Edge& e2)
{
  if (e1.getDestId() != e2.getSourceId()) {
    throw std::invalid_argument("Edges are not connected");
  }
  Edge shortcut{ e1.getSourceId(), e2.getDestId(), e1.getId(), e2.getId() };
  shortcut.setCost(e1.getCost() + e2.getCost());
  return shortcut;
}

std::future<std::vector<Edge>> Contractor::contract(MultiQueue& queue, Graph& g)
{
  return std::async(std::launch::async, ContractingThread{ queue, g, printStatistics });
}

std::set<NodePos> Contractor::independentSet(const Graph& g)
{
  std::set<NodePos> set;
  std::vector<std::pair<size_t, NodePos>> nodes;
  size_t nodeCount = g.getNodeCount();
  nodes.reserve(nodeCount);

  for (size_t i = 0; i < nodeCount; ++i) {
    NodePos p{ i };
    auto inEdges = g.getIngoingEdgesOf(p);
    auto outEdges = g.getOutgoingEdgesOf(p);
    size_t count = (inEdges.end() - inEdges.begin()) * (outEdges.end() - outEdges.begin());
    nodes.emplace_back(count, p);
  }
  std::sort(nodes.begin(), nodes.end());

  std::vector<bool> selected(nodeCount, true);

  for (size_t i = 0; i < nodeCount; ++i) {
    NodePos pos = nodes[i].second;

    if (selected[pos]) {
      for (const auto& inEdge : g.getIngoingEdgesOf(pos)) {
        selected[inEdge.end] = false;
      }
      for (const auto& outEdge : g.getOutgoingEdgesOf(pos)) {
        selected[outEdge.end] = false;
      }
      set.insert(pos);
    }
  }
  std::cout << "..."
            << "calculated greedy independent set of " << set.size() << "\n";
  return set;
}

std::set<NodePos> Contractor::reduce(std::set<NodePos>& set, const Graph& g)
{
  std::vector<std::pair<size_t, NodePos>> metric{};
  metric.reserve(set.size());

  std::transform(set.begin(), set.end(), std::back_inserter(metric), [&g](NodePos p) {
    auto inEdges = g.getIngoingEdgesOf(p);
    auto outEdges = g.getOutgoingEdgesOf(p);
    size_t count = (inEdges.end() - inEdges.begin()) * (outEdges.end() - outEdges.begin());
    return std::make_pair(count, p);
  });
  size_t divider = 4;
  auto median
      = metric.begin() + (metric.size() < divider ? metric.size() : metric.size() / divider);

  std::nth_element(metric.begin(), median, metric.end());

  std::set<NodePos> result{};
  std::transform(metric.begin(), median, std::inserter(result, result.begin()),
      [](auto pair) { return std::get<NodePos>(pair); });

  std::cout << "..."
            << "reduced greedy independent set to " << result.size() << "\n";
  return result;
}

void copyEdgesOfNode(Graph& g, NodePos pos, std::vector<EdgeId>& edges)
{
  auto outRange = g.getOutgoingEdgesOf(pos);
  std::transform(outRange.begin(), outRange.end(), std::back_inserter(edges),
      [](const auto& e) { return e.id; });
  auto inRange = g.getIngoingEdgesOf(pos);
  std::transform(
      inRange.begin(), inRange.end(), std::back_inserter(edges), [](const auto e) { return e.id; });
}

Graph Contractor::contract(Graph& g)
{
  auto start = std::chrono::high_resolution_clock::now();
  const int THREAD_COUNT
      = std::thread::hardware_concurrency() > 0 ? std::thread::hardware_concurrency() : 1;
  MultiQueue q{};
  std::vector<std::future<std::vector<Edge>>> futures;
  for (int i = 0; i < THREAD_COUNT; ++i) {
    futures.push_back(contract(q, g));
  }

  ++level;
  auto set = reduce(independentSet(g), g);
  std::vector<Node> nodes{};
  std::vector<EdgeId> edges{};
  std::vector<NodePos> nodesToContract{};
  nodesToContract.reserve(set.size());

  for (size_t i = 0; i < g.getNodeCount(); ++i) {
    NodePos pos{ i };
    if (set.find(pos) == set.end()) {
      nodes.push_back(g.getNode(pos));
      for (const auto& edge : g.getOutgoingEdgesOf(pos)) {
        if (set.find(edge.end) == set.end()) {
          edges.push_back(edge.id);
        }
      }
    } else {
      nodesToContract.push_back(pos);

      Node node = g.getNode(pos);
      node.assignLevel(level);

      contractedNodes.push_back(node);
      copyEdgesOfNode(g, pos, contractedEdges);
    }
  }

  size_t edgePairCount = 0;
  for (const auto& node : nodesToContract) {
    const auto& inEdges = g.getIngoingEdgesOf(node);
    const auto& outEdges = g.getOutgoingEdgesOf(node);
    for (const auto& in : inEdges) {
      for (const auto& out : outEdges) {
        if (in.end == out.end) {
          continue;
        }
        q.send(EdgePair{ in, out });
        ++edgePairCount;
      }
    }
  }
  q.close();

  if (printStatistics) {
    std::cout << "..." << edgePairCount << " edge pairs to contract" << '\n';
    StatisticsCollector::printHeader();
  }

  std::vector<Edge> shortcuts{};
  for (int i = 0; i < THREAD_COUNT; ++i) {
    auto shortcutsMsg = futures[i].get();
    std::move(shortcutsMsg.begin(), shortcutsMsg.end(), std::back_inserter(shortcuts));
  }
  Edge::administerEdges(shortcuts);
  std::transform(shortcuts.begin(), shortcuts.end(), std::back_inserter(edges),
      [](const auto& edge) { return edge.getId(); });

  auto end = std::chrono::high_resolution_clock::now();

  using s = std::chrono::seconds;
  std::cout << "..."
            << "Last contraction step took " << std::chrono::duration_cast<s>(end - start).count()
            << "s" << '\n';
  std::cout << "..."
            << "Created " << shortcuts.size() << " shortcuts." << '\n';
  shortcuts.clear();

  return Graph{ std::move(nodes), std::move(edges) };
}

Graph Contractor::mergeWithContracted(Graph& g)
{
  std::vector<Node> nodes{};
  std::vector<EdgeId> edges{};
  std::copy(contractedNodes.begin(), contractedNodes.end(), std::back_inserter(nodes));

  ++level;

  for (size_t i = 0; i < g.getNodeCount(); ++i) {
    NodePos pos{ i };
    auto node = g.getNode(pos);
    node.assignLevel(level);
    nodes.push_back(node);
    auto outEdges = g.getOutgoingEdgesOf(pos);
    std::transform(outEdges.begin(), outEdges.end(), std::back_inserter(edges),
        [](const auto& e) { return e.id; });
  }

  std::copy(contractedEdges.begin(), contractedEdges.end(), std::back_inserter(edges));

  std::cout << "Final graph has " << nodes.size() << " nodes and " << edges.size() << " edges."
            << '\n';
  return Graph{ std::move(nodes), std::move(edges) };
}

Graph Contractor::contractCompletely(Graph& g, double rest)
{

  Graph intermedG = contract(g);
  double uncontractedNodesPercent
      = std::round(intermedG.getNodeCount() * 10000.0 / g.getNodeCount()) / 100;
  std::cout << 100 - uncontractedNodesPercent << "% of the graph is contracted ("
            << intermedG.getNodeCount() << " nodes left)" << '\n'
            << std::flush;
  while (uncontractedNodesPercent > rest) {
    intermedG = contract(intermedG);
    uncontractedNodesPercent
        = std::round(intermedG.getNodeCount() * 10000.0 / g.getNodeCount()) / 100;
    std::cout << "..."
              << "total number of edges: " << intermedG.getEdgeCount() + contractedEdges.size()
              << "\n";
    std::cout << 100 - uncontractedNodesPercent << "% of the graph is contracted ("
              << intermedG.getNodeCount() << " nodes left)" << '\n'
              << std::flush;
  }
  std::cout << '\n';
  return mergeWithContracted(intermedG);
}
