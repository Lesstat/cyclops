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

#include <algorithm>
#include <cassert>
#include <iostream>

template <int Dim> std::vector<EdgeId> Edge<Dim>::internalId_vec;
template <int Dim> std::vector<NodeId> Edge<Dim>::source_vec;
template <int Dim> std::vector<NodeId> Edge<Dim>::destination_vec;
template <int Dim> std::vector<Cost<Dim>> Edge<Dim>::cost_vec;
template <int Dim> std::vector<ReplacedEdge> Edge<Dim>::edgeA_vec;
template <int Dim> std::vector<ReplacedEdge> Edge<Dim>::edgeB_vec;
template <int Dim> std::vector<NodePos> Edge<Dim>::sourcePos__vec;
template <int Dim> std::vector<NodePos> Edge<Dim>::destPos__vec;

template <int Dim>
Edge<Dim>::Edge(NodeId source, NodeId dest)
    : Edge(source, dest, {}, {})
{
}

template <int Dim>
Edge<Dim>::Edge(NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB)
    : source(source)
    , destination(dest)
    , edgeA(std::move(edgeA))
    , edgeB(std::move(edgeB))
{
  assert(source != dest);
  if (internalId_vec.size() > 0 && (edgeA || edgeB)) {
    if (Edge::getSourceId(*edgeA) == Edge::getSourceId(*edgeB)) {
      std::cerr << "Same starting point" << '\n';
      std::terminate();
    }
  }
}

template <int Dim> NodeId Edge<Dim>::getSourceId() const { return source; }
template <int Dim> NodeId Edge<Dim>::getDestId() const { return destination; }

template <int Dim> NodeId Edge<Dim>::getSourceId(EdgeId id) { return source_vec[id]; }
template <int Dim> NodeId Edge<Dim>::getDestId(EdgeId id) { return destination_vec[id]; }

template <int Dim> Edge<Dim> Edge<Dim>::createFromText(std::istream& text)
{

  uint32_t source, dest;
  CostD c;
  long edgeA, edgeB;

  text >> source >> dest;
  for (auto& c : c.values) {
    text >> c;
  }
  text >> edgeA >> edgeB;

  Edge e { NodeId(source), NodeId(dest) };
  if (edgeA > 0) {
    e.edgeA = EdgeId { static_cast<uint32_t>(edgeA) };
    e.edgeB = EdgeId { static_cast<uint32_t>(edgeB) };
  }
  e.cost = c;
  for (double c : e.cost.values) {
    if (0 > c) {
      throw std::invalid_argument("Cost below zero: " + std::to_string(c) + " for edge from "
          + std::to_string(source) + " to " + std::to_string(dest));
    }
  }
  return e;
}

template <int Dim> double Edge<Dim>::costByConfiguration(const ConfigD& conf) const
{
  return cost * conf;
}

template <int Dim> EdgeId Edge<Dim>::getId() const { return internalId; }

template <int Dim> const Cost<Dim>& Edge<Dim>::getCost() const { return cost; }

template <int Dim> const Cost<Dim>& Edge<Dim>::getCost(EdgeId id) { return cost_vec[id]; }

template <int Dim> void Edge<Dim>::setCost(CostD c) { this->cost = c; }

template <int Dim> const ReplacedEdge& Edge<Dim>::getEdgeA() const { return edgeA; }
template <int Dim> const ReplacedEdge& Edge<Dim>::getEdgeB() const { return edgeB; }
template <int Dim> const ReplacedEdge& Edge<Dim>::getEdgeA(EdgeId id) { return edgeA_vec[id]; }
template <int Dim> const ReplacedEdge& Edge<Dim>::getEdgeB(EdgeId id) { return edgeB_vec[id]; }

template <int Dim> HalfEdge<Dim> Edge<Dim>::makeHalfEdge(EdgeId id, NodePos, NodePos end)
{
  HalfEdgeD e;
  e.id = id;
  e.end = end;
  e.cost = getCost(id);
  return e;
}

template <int Dim> void Edge<Dim>::setId(EdgeId id) { this->internalId = id; }

template <int Dim> double HalfEdge<Dim>::costByConfiguration(const ConfigD& conf) const
{
  return cost * conf;
}
template <int Dim> std::vector<EdgeId> Edge<Dim>::administerEdges(std::vector<Edge>&& edges)
{
  std::vector<EdgeId> ids;
  ids.reserve(edges.size());
  for (size_t i = 0; i < edges.size(); ++i) {
    uint32_t new_id = internalId_vec.size() + i;
    auto& edge = edges[i];
    if (edge.getId() == 0) {
      edge.setId(EdgeId { new_id });
    } else if (edge.getId() != new_id) {
      std::cerr << "Edge ids dont align: " << '\n';
      std::terminate();
    }
    ids.emplace_back(new_id);
  }

  internalId_vec.reserve(internalId_vec.size() + edges.size());
  source_vec.reserve(source_vec.size() + edges.size());
  destination_vec.reserve(destination_vec.size() + edges.size());
  cost_vec.reserve(cost_vec.size() + edges.size());
  edgeA_vec.reserve(edgeA_vec.size() + edges.size());
  edgeB_vec.reserve(edgeB_vec.size() + edges.size());
  sourcePos__vec.reserve(sourcePos__vec.size() + edges.size());
  destPos__vec.reserve(destPos__vec.size() + edges.size());

  for (const auto& edge : edges) {

    internalId_vec.push_back(edge.internalId);
    source_vec.push_back(edge.source);
    destination_vec.push_back(edge.destination);
    cost_vec.push_back(edge.cost);
    edgeA_vec.push_back(edge.edgeA);
    edgeB_vec.push_back(edge.edgeB);
    sourcePos__vec.push_back(edge.sourcePos_);
    destPos__vec.push_back(edge.destPos_);
  }
  edges = std::vector<Edge>();
  return ids;
}
template <int Dim> const Edge<Dim> Edge<Dim>::getEdge(EdgeId id)
{
  Edge e(source_vec[id], destination_vec[id], edgeA_vec[id], edgeB_vec[id]);
  e.internalId = id;
  e.cost = cost_vec[id];
  e.sourcePos_ = sourcePos__vec[id];
  e.destPos_ = destPos__vec[id];
  return e;
}

template <int Dim> double Cost<Dim>::operator*(const ConfigD& conf) const
{
  double combinedCost = 0;

  for (size_t i = 0; i < Dim; ++i) {
    combinedCost += values[i] * conf.values[i];
  }

  if (!(combinedCost >= 0)) {
    std::cout << "Cost < 0 detected" << '\n';
    for (size_t i = 0; i < Dim; ++i) {
      std::cout << "metric " << i << ": " << values[i] << " * " << conf.values[i] << '\n';
    }
    throw std::invalid_argument("cost < 0");
  }
  return combinedCost;
}

template <int Dim> NodePos Edge<Dim>::sourcePos() const { return sourcePos_; }
template <int Dim> NodePos Edge<Dim>::destPos() const { return destPos_; }
template <int Dim> void Edge<Dim>::sourcePos(NodePos source) { sourcePos_ = source; }
template <int Dim> void Edge<Dim>::destPos(NodePos dest) { destPos_ = dest; }

template <int Dim> NodePos Edge<Dim>::sourcePos(EdgeId id) { return sourcePos__vec[id]; }
template <int Dim> NodePos Edge<Dim>::destPos(EdgeId id) { return destPos__vec[id]; }
template <int Dim> void Edge<Dim>::sourcePos(EdgeId id, NodePos source)
{
  sourcePos__vec[id] = source;
}
template <int Dim> void Edge<Dim>::destPos(EdgeId id, NodePos dest) { destPos__vec[id] = dest; }
