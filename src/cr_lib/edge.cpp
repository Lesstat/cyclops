#include "graph.hpp"
#include <iostream>
#include <sstream>
#include <vector>

size_t lastId = 0;

Edge::Edge(OsmId osmId, NodeId source, NodeId dest)
    : Edge(osmId, source, dest, {}, {})
{
}

Edge::Edge(OsmId osmId, NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB)
    : internalId(lastId++)
    , osmId(osmId)
    , source(source)
    , destination(dest)
    , sourcePos(0)
    , destPos(0)
    , edgeA(std::move(edgeA))
    , edgeB(std::move(edgeB))
{
}

Edge::~Edge() noexcept = default;

Edge::Edge(Edge&& other) noexcept
    : internalId(other.internalId)
    , sourcePos(other.sourcePos)
    , destPos(other.destPos)
{
  swap(other);
}

Edge& Edge::operator=(const Edge& other)
{
  Edge tmp{ other };
  swap(tmp);
  return *this;
}

Edge& Edge::operator=(Edge&& other) noexcept
{
  swap(other);
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

Edge Edge::createFromText(const std::string& text)
{
  std::stringstream ss{ text };

  size_t source, dest, type;
  double length, speed;
  long edgeA, edgeB;

  ss >> source >> dest >> length >> type >> speed >> edgeA >> edgeB;

  Edge e{ OsmId(0), NodeId(source), NodeId(dest) };
  if (edgeA > 0) {
    e.edgeA = edgeA;
    e.edgeB = edgeB;
  }
  e.cost.length = Length(length);
  return e;
}
