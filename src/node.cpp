#include "graph.hpp"

Node::Node(OsmId osmId, Lat lat, Lng lng, Height height)
    : osmId(osmId)
    , lat(lat)
    , lng(lng)
    , height(height)
{
}

Node::Node(const Node& other)
    : Node(other.osmId, other.lat, other.lng, other.height)
{
}

Node::Node(Node&& other) noexcept
{
  swap(other);
}

Node::~Node() noexcept {}

Node& Node::operator=(const Node& other)
{
  Node tmp{ other };
  swap(tmp);
  return *this;
}
Node& Node::operator=(Node&& other) noexcept
{
  swap(other);
  return *this;
}

size_t Node::getLevel() const
{
  return level;
}

void Node::assignLevel(size_t level)
{
  this->level = level;
}

void Node::swap(Node& other)
{
  std::swap(osmId, other.osmId);
  std::swap(lat, other.lat);
  std::swap(lng, other.lng);
  std::swap(height, other.height);
  std::swap(level, other.level);
}

OsmId Node::getOsmId() const
{
  return osmId;
}

std::ostream& operator<<(std::ostream& os, const Node& n)
{
  os << std::to_string(n.osmId.get());
  return os;
}
