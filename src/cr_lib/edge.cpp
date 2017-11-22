#include "graph.hpp"
#include <iostream>
#include <vector>

size_t lastId = 0;

Edge::Edge(OsmId id, NodeId source, NodeId dest)
    : Edge(id, source, dest, {}, {})
{
}

Edge::Edge(OsmId id, NodeId source, NodeId dest, const ReplacedEdge edgeA, const ReplacedEdge edgeB)
    : internalId(lastId++)
    , osmId(id)
    , source(source)
    , destination(dest)
    , sourcePos(0)
    , destPos(0)
    , edgeA(edgeA)
    , edgeB(edgeB)
{
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
    : Edge(other.osmId, other.source, other.destination, other.edgeA, other.edgeB)
{
}

void Edge::swap(Edge& other)
{
  std::swap(internalId, other.internalId);
  std::swap(osmId, other.osmId);
  std::swap(source, other.source);
  std::swap(destination, other.destination);
  std::swap(sourcePos, other.sourcePos);
  std::swap(destPos, other.destPos);
  std::swap(cost, other.cost);
  std::swap(edgeA, other.edgeA);
  std::swap(edgeB, other.edgeB);
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
  sourcePos = pos;
}
void Edge::setDestPos(size_t pos)
{
  destPos = pos;
}

size_t Edge::getSourcePos() const
{
  return sourcePos;
}
size_t Edge::getDestPos() const
{
  return destPos;
}
