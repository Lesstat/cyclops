#include "graph.hpp"
#include <iostream>
#include <vector>

size_t last_id = 0;

Edge::Edge(OsmId id, NodeId source, NodeId dest)
    : Edge(id, source, dest, {}, {})
{
}

Edge::Edge(OsmId id, NodeId source, NodeId dest, ReplacedEdge edge_a, ReplacedEdge edge_b)
    : osmId(id)
    , source(source)
    , destination(dest)
    , edge_a(edge_a)
    , edge_b(edge_b)
{

  internalId = last_id++;
}

Edge::~Edge() noexcept
{
}
Edge::Edge(Edge&& other) noexcept
{
  swap(other);
}

Edge& Edge::operator=(const Edge& other)
{
  Edge tmp{ other };
  swap(tmp);
  return *this;
}

Edge& Edge::operator=(Edge&& rhs) noexcept
{
  swap(rhs);
  return *this;
}

Edge::Edge(const Edge& other) noexcept
    : Edge(other.osmId, other.source, other.destination, other.edge_a, other.edge_b)
{
}

void Edge::swap(Edge& other)
{
  std::swap(internalId, other.internalId);
  std::swap(osmId, other.osmId);
  std::swap(source, other.source);
  std::swap(destination, other.destination);
  std::swap(source_pos, other.source_pos);
  std::swap(dest_pos, other.dest_pos);
  std::swap(cost, other.cost);
  std::swap(edge_a, other.edge_a);
  std::swap(edge_b, other.edge_b);
}

OsmId Edge::getSourceId() const
{
  return source;
}
OsmId Edge::getDestId() const
{
  return destination;
}
void Edge::setSourcePos(size_t pos)
{
  source_pos = pos;
}
void Edge::setDestPos(size_t pos)
{
  dest_pos = pos;
}

size_t Edge::getSourcePos() const
{
  return source_pos;
}
size_t Edge::getDestPos() const
{
  return dest_pos;
}
