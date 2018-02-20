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
#include <thread>

std::pair<bool, std::optional<RouteWithCount>> Contractor::isShortestPath(
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

Edge Contractor::createShortcut(const Edge& e1, const Edge& e2)
{
  if (e1.getDestId() != e2.getSourceId()) {
    throw std::invalid_argument("Edges are not connected");
  }
  Edge shortcut{ e1.getSourceId(), e2.getDestId(), e1.getId(), e2.getId() };
  shortcut.setCost(e1.getCost() + e2.getCost());
  return shortcut;
}

void addConstraint(const RouteWithCount& route, const Cost& c1, LinearProgram& lp)
{
  Cost c2{};
  for (const auto& edgeId : route.edges) {
    const auto& edge = Edge::getEdge(edgeId);
    c2 = c2 + edge.getCost();
  }
  Cost newCost = c1 - c2;
  lp.addConstraint({ newCost.length, static_cast<double>(newCost.height),
                       static_cast<double>(newCost.unsuitability) },
      0.0);
}

void Contractor::contract(MultiQueue& queue, Graph& g)
{
  std::thread t([this, &queue, &g]() {
    std::any msg;
    auto d = g.createNormalDijkstra();
    std::vector<Edge> shortcuts;
    shortcuts.reserve(g.getNodeCount());

    size_t shortCount = 0;
    size_t sameCount = 0;
    while (true) {
      queue.receive(msg);
      try {
        auto node = std::any_cast<NodePos>(msg);

        Config config{ LengthConfig{ 0.33 }, HeightConfig{ 0.33 }, UnsuitabilityConfig{ 0.33 } };
        const auto& inEdges = g.getIngoingEdgesOf(node);
        const auto& outEdges = g.getOutgoingEdgesOf(node);
        for (const auto& in : inEdges) {
          for (const auto& out : outEdges) {
            LinearProgram lp{ 3 };
            lp.objective({ 1.0, 1.0, 1.0 });
            lp.addConstraint({ 1.0, 1.0, 1.0 }, 1.0, 1.0);
            lp.addConstraint({ 1.0, 0.0, 0.0 }, 1.0, 0.001);
            lp.addConstraint({ 0.0, 1.0, 0.0 }, 1.0, 0.001);
            lp.addConstraint({ 0.0, 0.0, 1.0 }, 1.0, 0.001);

            Cost shortcutCost = in.cost + out.cost;

            while (true) {
              auto[isShortest, foundRoute] = isShortestPath(d, in, out, config);
              if (isShortest) {
                if (foundRoute->pathCount == 1) {
                  ++shortCount;
                  shortcuts.push_back(
                      Contractor::createShortcut(Edge::getEdge(in.id), Edge::getEdge(out.id)));
                  break;
                }
              }

              if (!foundRoute || foundRoute->edges.empty()) {
                break;
              }
              auto routes = d.routeIter(in.end, out.end);
            extraction:
              while (!routes.finished()) {
                auto optRoute = routes.next();
                if (!optRoute.has_value()) {
                  break;
                }
                auto route = *optRoute;
                if (route.edges.size() == 2 && route.edges[0] == in.id
                    && route.edges[1] == out.id) {
                  continue;
                }
                addConstraint(route, shortcutCost, lp);
              }

              if (!lp.solve()) {
                break;
              }
              auto values = lp.variableValues();
              if (values[0] + values[1] + values[2] > 2) {
                std::cout << "Cancelling to big" << '\n';
                break;
              }
              Config newConfig{ LengthConfig{ values[0] }, HeightConfig{ values[1] },
                UnsuitabilityConfig{ values[2] } };
              if (config == newConfig) {
                if (!routes.finished()) {
                  routes.doubleHeapsize();
                  goto extraction;
                }
                if (lp.exact()) {
                  ++sameCount;
                  shortcuts.push_back(
                      Contractor::createShortcut(Edge::getEdge(in.id), Edge::getEdge(out.id)));
                  break;
                }
                lp.exact(true);
              }
              config = newConfig;
            }
          }
        }
      } catch (std::bad_any_cast e) {
        auto responder = std::any_cast<std::shared_ptr<MultiQueue>>(msg);
        responder->send(shortcuts);
        std::cout << "short/same: " << shortCount << "/" << sameCount << '\n';
        return;
      }
    }
  });
  t.detach();
}

std::set<NodePos> Contractor::independentSet(const Graph& g)
{
  std::set<NodePos> set;
  size_t nodeCount = g.getNodeCount();
  std::vector<bool> selected(nodeCount, true);

  for (size_t i = 0; i < nodeCount; ++i) {
    NodePos pos{ i };
    if (selected[i]) {
      for (const auto& inEdge : g.getIngoingEdgesOf(pos)) {
        selected[inEdge.end] = false;
      }
      for (const auto& outEdge : g.getOutgoingEdgesOf(pos)) {
        selected[outEdge.end] = false;
      }
      set.insert(pos);
    }
  }

  return set;
}

std::set<NodePos> Contractor::reduce(std::set<NodePos>& set, const Graph& g)
{
  std::vector<std::pair<NodePos, size_t>> metric{};
  metric.reserve(set.size());

  std::transform(set.begin(), set.end(), std::back_inserter(metric), [&g](NodePos p) {
    auto inEdges = g.getIngoingEdgesOf(p);
    auto outEdges = g.getOutgoingEdgesOf(p);
    size_t count = (inEdges.end() - inEdges.begin()) * (outEdges.end() - outEdges.begin());
    return std::make_pair(p, count);
  });

  auto median = metric.begin() + (metric.size() == 1 ? 1 : metric.size() / 2);

  std::nth_element(metric.begin(), median, metric.end());

  std::set<NodePos> result{};
  std::transform(metric.begin(), median, std::inserter(result, result.begin()),
      [](auto pair) { return std::get<NodePos>(pair); });

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
  for (int i = 0; i < THREAD_COUNT; ++i) {
    contract(q, g);
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
  std::sort(begin(nodesToContract), end(nodesToContract), [&g](const auto& pos1, const auto& pos2) {
    return g.getInTimesOutDegree(pos1) > g.getInTimesOutDegree(pos2);
  });
  for (const auto& node : nodesToContract) {
    q.send(std::any{ node });
  }

  auto back = std::make_shared<MultiQueue>();
  for (int i = 0; i < THREAD_COUNT; ++i) {
    q.send(std::any{ back });
  }
  std::vector<Edge> shortcuts{};
  for (int i = 0; i < THREAD_COUNT; ++i) {
    std::any msg;
    back->receive(msg);
    auto shortcutsMsg = std::any_cast<std::vector<Edge>>(msg);
    std::move(shortcutsMsg.begin(), shortcutsMsg.end(), std::back_inserter(shortcuts));
  }
  Edge::administerEdges(shortcuts);
  std::transform(shortcuts.begin(), shortcuts.end(), std::back_inserter(edges),
      [](const auto& edge) { return edge.getId(); });

  auto end = std::chrono::high_resolution_clock::now();

  using s = std::chrono::seconds;
  std::cout << "Last contraction step took " << std::chrono::duration_cast<s>(end - start).count()
            << "s" << '\n';
  std::cout << "Created " << shortcuts.size() << " shortcuts." << '\n';
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

Graph Contractor::contractCompletely(Graph& g, unsigned short rest)
{

  Graph intermedG = contract(g);
  int uncontractedNodesPercent = intermedG.getNodeCount() * 100 / g.getNodeCount();
  std::cout << 100 - uncontractedNodesPercent << "% of the graph is contracted" << '\n'
            << std::flush;
  while (uncontractedNodesPercent > rest) {
    intermedG = contract(intermedG);
    uncontractedNodesPercent = intermedG.getNodeCount() * 100 / g.getNodeCount();
    std::cout << 100 - uncontractedNodesPercent << "% of the graph is contracted" << '\n'
              << std::flush;
  }
  std::cout << '\n';
  return mergeWithContracted(intermedG);
}
