#include "graph.hpp"
#include <sstream>

Node::Node(OsmId osmId, Lat lat, Lng lng, Height height)
    : osmId(osmId)
    , lat(lat)
    , lng(lng)
    , height(height)
    , level(0)
{
}

Node::Node(const Node& other)
    : Node(other.osmId, other.lat, other.lng, other.height)
{
}

Node::Node(Node&& other) noexcept
    : level(other.level)
{
  swap(other);
}

Node::~Node() noexcept = default;

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
  os << std::to_string(n.osmId);
  return os;
}

Node Node::createFromText(const std::string& text)
{
  std::stringstream ss{ text };
  size_t id, osmId, level;
  double lat, lng, height;
  ss >> id >> osmId >> lat >> lng >> height >> level;

  Node n{ OsmId(osmId), Lat(lat), Lng(lng), Height(height) };
  n.level = level;
  return n;
}
