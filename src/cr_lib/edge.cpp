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
#include "graph.hpp"
#include <algorithm>
#include <cassert>

std::vector<EdgeId> Edge::internalId_vec;
std::vector<NodeId> Edge::source_vec;
std::vector<NodeId> Edge::destination_vec;
std::vector<Cost> Edge::cost_vec;
std::vector<ReplacedEdge> Edge::edgeA_vec;
std::vector<ReplacedEdge> Edge::edgeB_vec;
std::vector<NodePos> Edge::sourcePos__vec;
std::vector<NodePos> Edge::destPos__vec;

Edge::Edge(NodeId source, NodeId dest)
    : Edge(source, dest, {}, {})
{
}

Edge::Edge(NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB)
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

NodeId Edge::getSourceId() const { return source; }
NodeId Edge::getDestId() const { return destination; }

NodeId Edge::getSourceId(EdgeId id) { return source_vec[id]; }
NodeId Edge::getDestId(EdgeId id) { return destination_vec[id]; }

Edge Edge::createFromText(std::istream& text)
{

  size_t source, dest;
  Cost c;
  long edgeA, edgeB;

  text >> source >> dest;
  for (auto& c : c.values) {
    text >> c;
  }
  text >> edgeA >> edgeB;

  Edge e { NodeId(source), NodeId(dest) };
  if (edgeA > 0) {
    e.edgeA = EdgeId { static_cast<size_t>(edgeA) };
    e.edgeB = EdgeId { static_cast<size_t>(edgeB) };
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

double Edge::costByConfiguration(const Config& conf) const { return cost * conf; }

EdgeId Edge::getId() const { return internalId; }

const Cost& Edge::getCost() const { return cost; }

const Cost& Edge::getCost(EdgeId id) { return cost_vec[id]; }

void Edge::setCost(Cost c) { this->cost = c; }

const ReplacedEdge& Edge::getEdgeA() const { return edgeA; }
const ReplacedEdge& Edge::getEdgeB() const { return edgeB; }

const ReplacedEdge& Edge::getEdgeA(EdgeId id) { return edgeA_vec[id]; }
const ReplacedEdge& Edge::getEdgeB(EdgeId id) { return edgeB_vec[id]; }

HalfEdge Edge::makeHalfEdge(EdgeId id, NodePos, NodePos end)
{
  HalfEdge e;
  e.id = id;
  e.end = end;
  e.cost = getCost(id);
  return e;
}

void Edge::setId(EdgeId id) { this->internalId = id; }

double HalfEdge::costByConfiguration(const Config& conf) const { return cost * conf; }

std::vector<EdgeId> Edge::administerEdges(std::vector<Edge>&& edges)
{
  std::vector<EdgeId> ids;
  ids.reserve(edges.size());
  for (size_t i = 0; i < edges.size(); ++i) {
    size_t new_id = internalId_vec.size() + i;
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
const Edge Edge::getEdge(EdgeId id)
{
  Edge e(source_vec[id], destination_vec[id], edgeA_vec[id], edgeB_vec[id]);
  e.internalId = id;
  e.cost = cost_vec[id];
  e.sourcePos_ = sourcePos__vec[id];
  e.destPos_ = destPos__vec[id];
  return e;
}

double Cost::operator*(const Config& conf) const
{
  double combinedCost = 0;

  for (size_t i = 0; i < dim; ++i) {
    combinedCost += values[i] * conf.values[i];
  }

  if (!(combinedCost >= 0)) {
    std::cout << "Cost < 0 detected" << '\n';
    for (size_t i = 0; i < dim; ++i) {
      std::cout << "metric " << i << ": " << values[i] << " * " << conf.values[i] << '\n';
    }
    throw std::invalid_argument("cost < 0");
  }
  return combinedCost;
}

NodePos Edge::sourcePos() const { return sourcePos_; }
NodePos Edge::destPos() const { return destPos_; }
void Edge::sourcePos(NodePos source) { sourcePos_ = source; }
void Edge::destPos(NodePos dest) { destPos_ = dest; }

NodePos Edge::sourcePos(EdgeId id) { return sourcePos__vec[id]; }
NodePos Edge::destPos(EdgeId id) { return destPos__vec[id]; }
void Edge::sourcePos(EdgeId id, NodePos source) { sourcePos__vec[id] = source; }
void Edge::destPos(EdgeId id, NodePos dest) { destPos__vec[id] = dest; }
