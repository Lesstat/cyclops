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
    NormalDijkstra& d, const EdgeId& startEdgeId, const EdgeId& destEdgeId, const Config& conf)
{

  const auto& startEdge = Edge::getEdge(startEdgeId);
  const auto& destEdge = Edge::getEdge(destEdgeId);

  auto foundRoute = d.findBestRoute(startEdge.getSourcePos(), destEdge.getDestPos(), conf);
  if (!foundRoute) {
    return std::make_pair(false, foundRoute);
  }

  auto route = foundRoute.value();
  bool isShortest = route.edges.size() == 2 && route.edges[0].getId() == startEdgeId
      && route.edges[1].getId() == destEdgeId;
  return std::make_pair(isShortest, foundRoute);
}

Edge Contractor::createShortcut(const Edge& e1, const Edge& e2)
{
  if (e1.getDestId() != e2.getSourceId()) {
    throw std::invalid_argument("Edges are not connected");
  }
  Edge shortcut{ e1.getSourceId(), e2.getDestId(), e1.getId(), e2.getId() };
  shortcut.setSourcePos(e1.getSourcePos());
  shortcut.setDestPos(e1.getDestPos());
  shortcut.setCost(e1.getCost() + e2.getCost());
  return shortcut;
}

void Contractor::contract(MultiQueue& queue, Graph& g)
{
  std::thread t([this, &queue, &g]() {
    std::any msg;
    auto d = g.createNormalDijkstra();
    std::vector<Edge> shortcuts;
    shortcuts.reserve(g.getNodeCount());

    while (true) {
      queue.receive(msg);
      try {
        auto node = std::any_cast<NodePos>(msg);

        Config config{ LengthConfig{ 0.33 }, HeightConfig{ 0.33 }, UnsuitabilityConfig{ 0.33 } };
        const auto& inEdges = g.getIngoingEdgesOf(node);
        const auto& outEdges = g.getOutgoingEdgesOf(node);
        for (const auto& inId : inEdges) {
          for (const auto& outId : outEdges) {
            LinearProgram lp = LinearProgram::setUpLPForContraction();
            const auto& in = Edge::getEdge(inId);
            const auto& out = Edge::getEdge(outId);

            Cost c1 = in.getCost() + out.getCost();

            while (true) {
              auto[isShortest, foundRoute] = isShortestPath(d, in.getId(), out.getId(), config);
              if (isShortest) {
                if (foundRoute->pathCount == 1) {
                  shortcuts.push_back(Contractor::createShortcut(
                      Edge::getEdge(in.getId()), Edge::getEdge(out.getId())));
                  break;
                }
                foundRoute = d.findOtherRoute(*foundRoute);
              }

              if (!foundRoute || foundRoute->edges.empty()) {
                break;
              }
              Cost c2{};
              for (const auto& edge : foundRoute->edges) {
                c2 = c2 + edge.getCost();
              }
              Cost newCost = c1 - c2;
              lp.addConstraint({ newCost.length, static_cast<double>(newCost.height),
                                   static_cast<double>(newCost.unsuitability) },
                  0.0);

              if (!lp.solve()) {
                break;
              }
              auto values = lp.variableValues();
              Config newConfig{ LengthConfig{ values[0] }, HeightConfig{ values[1] },
                UnsuitabilityConfig{ values[2] } };
              if (config == newConfig) {
                shortcuts.push_back(Contractor::createShortcut(
                    Edge::getEdge(in.getId()), Edge::getEdge(out.getId())));
                break;
              }
              config = newConfig;
            }
          }
        }
      } catch (std::bad_any_cast e) {
        auto responder = std::any_cast<std::shared_ptr<MultiQueue>>(msg);
        responder->send(shortcuts);
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
      for (const auto& inEdgeId : g.getIngoingEdgesOf(pos)) {
        const auto& inEdge = Edge::getEdge(inEdgeId);
        selected[inEdge.getSourcePos()] = false;
      }
      for (const auto& outEdgeId : g.getOutgoingEdgesOf(pos)) {
        const auto& outEdge = Edge::getEdge(outEdgeId);
        selected[outEdge.getDestPos()] = false;
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
  std::copy(outRange.begin(), outRange.end(), std::back_inserter(edges));
  auto inRange = g.getIngoingEdgesOf(pos);
  std::copy(inRange.begin(), inRange.end(), std::back_inserter(edges));
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

  for (size_t i = 0; i < g.getNodeCount(); ++i) {
    NodePos pos{ i };
    if (set.find(pos) == set.end()) {
      nodes.push_back(g.getNode(pos));
      for (const auto& edgeId : g.getOutgoingEdgesOf(pos)) {
        const auto& edge = Edge::getEdge(edgeId);
        if (set.find(edge.getDestPos()) == set.end()) {
          edges.push_back(edgeId);
        }
      }
    } else {
      q.send(std::any{ pos });

      Node node = g.getNode(pos);
      node.assignLevel(level);

      contractedNodes.push_back(node);
      copyEdgesOfNode(g, pos, contractedEdges);
    }
  }
  auto back = std::make_shared<MultiQueue>();
  for (int i = 0; i < THREAD_COUNT; ++i) {
    q.send(std::any{ back });
  }
  size_t shortcutCount = 0;
  for (int i = 0; i < THREAD_COUNT; ++i) {
    std::any msg;
    back->receive(msg);
    auto shortcuts = std::any_cast<std::vector<Edge>>(msg);
    shortcutCount += shortcuts.size();
    Edge::administerEdges(shortcuts);
    std::transform(shortcuts.begin(), shortcuts.end(), std::back_inserter(edges),
        [](const auto& edge) { return edge.getId(); });
  }

  auto end = std::chrono::high_resolution_clock::now();

  using s = std::chrono::seconds;
  std::cout << "Last contraction step took " << std::chrono::duration_cast<s>(end - start).count()
            << "s" << '\n';
  std::cout << "Created " << shortcutCount << " shortcuts." << '\n';
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
    std::copy(outEdges.begin(), outEdges.end(), std::back_inserter(edges));
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
