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
#include "graph.hpp"
#include "ndijkstra.hpp"

void printEdge(std::ofstream& dotFile, const HalfEdge& edge, const std::set<EdgeId>& route1Edges,
    const std::set<EdgeId>& route2Edges, const Config& config, const Graph& g)
{
  dotFile << '"' << edge.begin << "(" << g.getLevelOf(edge.begin) << ")" << '"';
  dotFile << " -> ";
  dotFile << '"' << edge.end << "(" << g.getLevelOf(edge.end) << ")" << '"';
  dotFile << " [label = \"";
  bool first = true;
  for (auto& c : edge.cost.values) {
    if (!first)
      dotFile << ", ";
    dotFile << c;
    first = false;
  }
  dotFile << " | " << edge.cost * config << "\"";
  dotFile << " color = ";
  bool partOfShortcut = route2Edges.count(edge.id) > 0;
  bool partOfRoute = route1Edges.count(edge.id) > 0;
  if (partOfRoute && partOfShortcut) {
    dotFile << "green";
  } else if (partOfShortcut) {
    dotFile << "blue";
  } else if (partOfRoute) {
    dotFile << "yellow";
  } else {
    dotFile << "black";
  }
  dotFile << " ]" << '\n';
}

void printRoutes(std::ofstream& dotFile, const Graph& graph, const RouteWithCount& route1,
    const Route& route2, const Config& config)
{
  dotFile << "digraph G{" << '\n';
  dotFile << "rankdir=LR;" << '\n';
  dotFile << "size=8;" << '\n';

  auto from = Edge::getEdge(route1.edges.front()).sourcePos();
  auto to = Edge::getEdge(route1.edges.back()).destPos();
  dotFile << "node[ shape = doublecircle color = red]; " << '"' << from << "("
          << graph.getLevelOf(from) << ")" << '"';
  dotFile << " " << '"' << to << "(" << graph.getLevelOf(to) << ")" << '"' << ";" << '\n';
  dotFile << "node[ shape = circle color = black]; " << '\n';

  std::set<EdgeId> printedEdges;
  std::set<EdgeId> route1Edges;
  std::copy(
      route1.edges.begin(), route1.edges.end(), std::inserter(route1Edges, route1Edges.begin()));

  std::set<EdgeId> route2Edges;
  std::transform(route2.edges.begin(), route2.edges.end(),
      std::inserter(route2Edges, route2Edges.begin()),
      [](const auto& edge) { return edge.getId(); });

  for (auto& routeEdgeId : route1.edges) {
    auto& routeEdge = Edge::getEdge(routeEdgeId);
    auto node = routeEdge.sourcePos();
    for (auto& edge : graph.getOutgoingEdgesOf(node)) {
      if (printedEdges.count(edge.id) == 0) {
        printedEdges.insert(edge.id);
        printEdge(dotFile, edge, route1Edges, route2Edges, config, graph);
      }
      for (auto& edge2 : graph.getOutgoingEdgesOf(edge.end)) {
        if (printedEdges.count(edge2.id) == 0) {
          printedEdges.insert(edge2.id);
          printEdge(dotFile, edge2, route1Edges, route2Edges, config, graph);
        }
      }
    }
  }

  for (auto& edge : graph.getIngoingEdgesOf(to)) {
    if (printedEdges.count(edge.id) == 0) {
      printedEdges.insert(edge.id);
      HalfEdge e;
      e.id = edge.id;
      e.begin = edge.end;
      e.end = edge.begin;
      e.cost = edge.cost;
      printEdge(dotFile, e, route1Edges, route2Edges, config, graph);
    }
  }

  for (auto& routeEdge : route2.edges) {
    for (auto& edge : graph.getOutgoingEdgesOf(routeEdge.sourcePos())) {
      if (printedEdges.count(edge.id) == 0) {
        printedEdges.insert(edge.id);
        printEdge(dotFile, edge, route1Edges, route2Edges, config, graph);
      }
      for (auto& edge2 : graph.getOutgoingEdgesOf(edge.end)) {
        if (printedEdges.count(edge2.id) == 0) {
          printedEdges.insert(edge2.id);
          printEdge(dotFile, edge2, route1Edges, route2Edges, config, graph);
        }
      }
    }
  }
  dotFile << "}" << '\n';
  dotFile.flush();
}
