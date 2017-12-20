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
#include <sstream>
#include <vector>

size_t lastId = 0;

Edge::Edge(NodeId source, NodeId dest)
    : Edge(source, dest, {}, {})
{
}

Edge::Edge(NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB)
    : internalId(lastId++)
    // , osmId(osmId)
    , source(source)
    , destination(dest)
    , edgeA(std::move(edgeA))
    , edgeB(std::move(edgeB))
{
}

Edge::~Edge() noexcept = default;

Edge::Edge(Edge&& other) noexcept
    : internalId(other.internalId)
{
  swap(other);
}

Edge& Edge::operator=(const Edge& other) = default;

Edge& Edge::operator=(Edge&& other) noexcept = default;

Edge::Edge(const Edge& other) = default;

void Edge::swap(Edge& other)
{
  std::swap(internalId, other.internalId);
  std::swap(source, other.source);
  std::swap(destination, other.destination);
  std::swap(sourcePos, other.sourcePos);
  std::swap(destinationPos, other.destinationPos);
  std::swap(cost, other.cost);
  std::swap(edgeA, other.edgeA);
  std::swap(edgeB, other.edgeB);
}

NodeId Edge::getSourceId() const
{
  return source;
}
NodeId Edge::getDestId() const
{
  return destination;
}
NodePos Edge::getSourcePos() const
{
  return sourcePos;
}
NodePos Edge::getDestPos() const
{
  return destinationPos;
}

void Edge::setDestPos(NodePos p)
{
  destinationPos = p;
}

void Edge::setSourcePos(NodePos p)
{
  sourcePos = p;
}

Edge Edge::createFromText(const std::string& text)
{

  size_t source, dest;
  double length;
  short height, unsuitability;
  long edgeA, edgeB;

  std::sscanf(text.c_str(), "%lu%lu%lf%hd%hd%li%li", &source, &dest, &length, &height, &unsuitability, &edgeA, &edgeB); //NOLINT

  Edge e{ NodeId(source), NodeId(dest) };
  if (edgeA > 0) {
    e.edgeA = EdgeId{ static_cast<size_t>(edgeA) };
    e.edgeB = EdgeId{ static_cast<size_t>(edgeB) };
  }
  e.cost.length = Length(length);
  e.cost.height = Height(height);
  e.cost.unsuitability = Unsuitability(unsuitability);
  return e;
}

double Edge::costByConfiguration(const Config& conf) const
{
  return cost.length * conf.length
      + cost.height * conf.height
      + cost.unsuitability * conf.unsuitability;
}

EdgeId Edge::getId() const
{
  return internalId;
}

const Cost& Edge::getCost() const
{
  return cost;
}

void Edge::setCost(Cost c)
{
  this->cost = c;
}
