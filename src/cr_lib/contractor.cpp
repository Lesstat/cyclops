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
#include "dijkstra.hpp"
#include "linearProgram.hpp"

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

bool Contractor::isShortestPath(Graph& g, const EdgeId& startEdgeId,
    const EdgeId& destEdgeId, const Config& conf)
{
  if (!dijkstra) {
    dijkstra = g.createDijkstra();
  }
  const auto& startEdge = g.getEdge(startEdgeId);
  const auto& destEdge = g.getEdge(destEdgeId);

  foundRoute = dijkstra->findBestRoute(startEdge.getSourcePos(), destEdge.getDestPos(), conf);
  if (!foundRoute) {
    return false;
  }
  auto route = foundRoute.value();
  return route.edges.size() == 2 && route.edges[0].getId() == startEdgeId && route.edges[1].getId() == destEdgeId;
}

std::vector<Edge> Contractor::contract(Graph& g, const NodePos& node)
{
  std::vector<Edge> shortcuts;
  Config config{ LengthConfig{ 0.33 }, HeightConfig{ 0.33 },
    UnsuitabilityConfig{ 0.33 } };
  auto d = g.createDijkstra();
  const auto& inEdges = g.getIngoingEdgesOf(node);
  const auto& outEdges = g.getOutgoingEdgesOf(node);
  for (const auto& in : inEdges) {
    for (const auto& out : outEdges) {
      LinearProgram lp{ 3 };
      lp.objective({ 1.0, 1.0, 1.0 });
      lp.addConstraint({ 1.0, 1.0, 1.0 }, 1.0, GLP_FX);
      while (true) {
        const auto& inEdge = g.getEdge(in);
        const auto& outEdge = g.getEdge(out);

        if (isShortestPath(g, in, out, config)) {
          shortcuts.push_back(createShortcut(inEdge, outEdge));
          break;
        }

        if (!foundRoute) {
          break;
        }
        Cost c1 = inEdge.getCost() + outEdge.getCost();
        Cost c2{};
        for (const auto& edge : foundRoute->edges) {
          c2 = c2 + edge.getCost();
        }
        Cost newCost = c1 - c2;
        lp.addConstraint({ newCost.length, static_cast<double>(newCost.height), static_cast<double>(newCost.unsuitability) }, 0.0);

        if (!lp.solve()) {
          break;
        }
        auto values = lp.variableValues();
        config = Config{ LengthConfig{ values[0] }, HeightConfig{ values[1] }, UnsuitabilityConfig{ values[2] } };
      }
    }
  }

  return shortcuts;
}

std::vector<NodePos> Contractor::independentSet(const Graph& g)
{
  std::vector<NodePos> set;
  size_t nodeCount = g.getNodeCount();
  std::vector<bool> selected(nodeCount, true);

  for (size_t i = 0; i < nodeCount; ++i) {
    NodePos pos{ i };
    if (selected[i]) {
      for (const auto& inEdgeId : g.getIngoingEdgesOf(pos)) {
        const auto& inEdge = g.getEdge(inEdgeId);
        selected[inEdge.getSourcePos()] = false;
      }
      for (const auto& outEdgeId : g.getOutgoingEdgesOf(pos)) {
        const auto& outEdge = g.getEdge(outEdgeId);
        selected[outEdge.getDestPos()] = false;
      }
      set.push_back(pos);
    }
  }

  return set;
}
