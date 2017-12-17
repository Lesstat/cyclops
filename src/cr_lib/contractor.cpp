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

Edge Contractor::createShortcut(const Edge& e1, const Edge& e2)
{
  if (e1.getDestId() != e2.getSourceId()) {
    throw std::invalid_argument("Edges are not connected");
  }
  Edge shortcut{ e1.getSourceId(), e2.getDestId(), e1.getId(), e2.getId() };
  shortcut.setCost(e1.getCost() + e2.getCost());
  return shortcut;
}

bool Contractor::isShortestPath(const Graph& g, const EdgeId& startEdgeId, const EdgeId& destEdgeId, const Config& conf)
{
  Dijkstra d = g.createDijkstra();
  const auto& startEdge = g.getEdge(startEdgeId);
  const auto& destEdge = g.getEdge(destEdgeId);

  auto maybeRoute = d.findBestRoute(startEdge.getSourceId(), destEdge.getDestId(), conf);
  if (!maybeRoute.has_value()) {
    return false;
  }
  auto route = maybeRoute.value();
  return route.edges.size() == 2 && route.edges[0].getId() == startEdgeId && route.edges[1].getId() == destEdgeId;
}

std::vector<Edge> Contractor::contract(const Graph& g, const NodeId& node)
{
  std::vector<Edge> shortcuts;
  return shortcuts;
}
