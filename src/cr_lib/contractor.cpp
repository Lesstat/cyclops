
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
