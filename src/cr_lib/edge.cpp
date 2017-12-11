#include "dijkstra.hpp"
#include "graph.hpp"
#include <iostream>
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

Edge::Edge(const Edge& other) = default;

void Edge::swap(Edge& other)
{
  std::swap(internalId, other.internalId);
  std::swap(source, other.source);
  std::swap(destination, other.destination);
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
