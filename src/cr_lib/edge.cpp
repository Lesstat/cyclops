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

std::atomic<size_t> Edge::lastId = 0;
std::vector<Edge> Edge::edges{};

Edge::Edge(NodeId source, NodeId dest)
    : Edge(source, dest, {}, {})
{
}

Edge::Edge(NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB)
    : internalId(lastId++)
    , source(source)
    , destination(dest)
    , edgeA(std::move(edgeA))
    , edgeB(std::move(edgeB))
{
  assert(source != dest);
}

NodeId Edge::getSourceId() const { return source; }
NodeId Edge::getDestId() const { return destination; }

Edge Edge::createFromText(const std::string& text)
{

  size_t source, dest;
  double length, height, unsuitability;
  long edgeA, edgeB;

  std::sscanf(text.c_str(), "%lu%lu%lf%lf%lf%li%li", &source, &dest, &length, &height, // NOLINT
      &unsuitability, &edgeA, &edgeB); // NOLINT

  Edge e{ NodeId(source), NodeId(dest) };
  if (edgeA > 0) {
    e.edgeA = EdgeId{ static_cast<size_t>(edgeA) };
    e.edgeB = EdgeId{ static_cast<size_t>(edgeB) };
  }
  e.cost = Cost({ length, height, unsuitability });
  for (double c : e.cost.values) {
    assert(0 <= c);
  }
  return e;
}

double Edge::costByConfiguration(const Config& conf) const { return cost * conf; }

EdgeId Edge::getId() const { return internalId; }

const Cost& Edge::getCost() const { return cost; }

void Edge::setCost(Cost c) { this->cost = c; }

const ReplacedEdge& Edge::getEdgeA() const { return edgeA; }
const ReplacedEdge& Edge::getEdgeB() const { return edgeB; }

HalfEdge Edge::makeHalfEdge(NodePos begin, NodePos end) const
{
  HalfEdge e;
  e.id = internalId;
  e.begin = begin;
  e.end = end;
  e.cost = cost;
  return e;
}

double HalfEdge::costByConfiguration(const Config& conf) const { return cost * conf; }

void Edge::administerEdges(const std::vector<Edge>& edges)
{
  Edge::edges.reserve(lastId);
  for (const auto& edge : edges) {
    while (Edge::edges.size() <= edge.getId()) {
      Edge::edges.emplace_back(Edge{});
    }
    Edge::edges[edge.getId()] = edge;
  }
}
const Edge& Edge::getEdge(EdgeId id) { return edges.at(id); }
Edge& Edge::getMutEdge(EdgeId id) { return edges.at(id); }

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
  if (combinedCost == 0) {
    return std::numeric_limits<double>::epsilon();
  }
  return combinedCost;
}

NodePos Edge::sourcePos() const { return sourcePos_; }
NodePos Edge::destPos() const { return destPos_; }
void Edge::sourcePos(NodePos source) { sourcePos_ = source; }
void Edge::destPos(NodePos dest) { destPos_ = dest; }
