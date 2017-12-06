#ifndef GRAPH_H
#define GRAPH_H

#include "namedType.hpp"
#include <cstdlib>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

using OsmId = NamedType<size_t, struct OsmIdParameter>;
using NodeId = NamedType<size_t, struct OsmIdParameter>;
using EdgeId = NamedType<size_t, struct EdgeParameter>;
using Lat = NamedType<double, struct LatParameter>;
using Lng = NamedType<double, struct LngParameter>;
using Height = NamedType<double, struct HeightParameter>;
using Length = NamedType<double, struct LengthParameter>;
using Unsuitability = NamedType<double, struct UnsuitabilityParameter>;

class Dijkstra;
struct Config;

struct Cost {
  Length length;
  Height height;
  Unsuitability unsuitability;
  Cost(Length l, Height h, Unsuitability u)
      : length(l)
      , height(h)
      , unsuitability(u)
  {
  }
  Cost()
      : length(0)
      , height(0)
      , unsuitability(0)
  {
  }

  Cost operator+(Cost c)
  {
    return Cost{ Length(length.get() + c.length.get()), Height(height.get() + c.height.get()), Unsuitability(unsuitability.get() + c.unsuitability.get()) };
  };
};

struct NodeOffset {
  size_t in{ 0 };
  size_t out{ 0 };
  NodeOffset() = default;
  NodeOffset(size_t in, size_t out)
      : in(in)
      , out(out)
  {
  }
};

using ReplacedEdge = std::optional<size_t>;

class Edge {
  public:
  Edge(OsmId osmId, NodeId source, NodeId dest);
  Edge(OsmId osmId, NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB);
  Edge(const Edge& other);
  Edge(Edge&& other) noexcept;
  virtual ~Edge() noexcept;
  Edge& operator=(const Edge& other);
  Edge& operator=(Edge&& other) noexcept;

  NodeId getSourceId() const;
  NodeId getDestId() const;

  EdgeId getId() const;
  const Cost& getCost() const;
  double costByConfiguration(const Config& conf) const;

  static Edge createFromText(const std::string& text);

  friend void testEdgeInternals(const Edge& e,
      size_t source,
      size_t destination,
      double length,
      double height,
      double unsuitability,
      const ReplacedEdge& edgeA,
      const ReplacedEdge& edgeB);

  private:
  void swap(Edge& other);

  EdgeId internalId;
  OsmId osmId;
  NodeId source;
  NodeId destination;
  Cost cost;
  ReplacedEdge edgeA;
  ReplacedEdge edgeB;
};

class Node {
  public:
  Node(OsmId osmId, Lat lat, Lng lng, Height height);
  Node(const Node& other);
  Node(Node&& other) noexcept;
  virtual ~Node() noexcept;
  Node& operator=(const Node& other);
  Node& operator=(Node&& other) noexcept;

  void assignLevel(size_t level);
  size_t getLevel() const;
  OsmId getOsmId() const;
  friend std::ostream& operator<<(std::ostream& os, const Node& n);

  static Node createFromText(const std::string& text);
  friend void testNodeInternals(const Node& n, size_t osmId, double lat, double lng, size_t level);

  private:
  void swap(Node& other);

  OsmId osmId;
  Lat lat;
  Lng lng;
  Height height;
  size_t level;
};

class Graph {
  public:
  Graph(std::vector<Node>&& nodes, std::vector<Edge>&& edges);
  Graph(const Graph& other);
  Graph(Graph&& other) noexcept;
  virtual ~Graph() noexcept;
  Graph& operator=(const Graph& other);
  Graph& operator=(Graph&& other) noexcept;

  friend std::ostream& operator<<(std::ostream& /*s*/, const Graph& /*g*/);

  std::vector<NodeOffset> const& getOffsets() const;
  Dijkstra createDijkstra() const;

  using EdgeRange = std::pair<std::vector<Edge>::const_iterator, std::vector<Edge>::const_iterator>;
  EdgeRange getOutgoingEdgesOf(NodeId n) const;
  EdgeRange getIngoingEdgesOf(NodeId n) const;

  size_t getLevelOf(NodeId n) const;
  const Edge& getEdge(EdgeId e) const;

  static Graph createFromStream(std::istream& file);

  const Node& getNode(NodeId id) const;

  private:
  void swap(Graph& other);

  std::vector<Node> nodes;
  std::vector<NodeOffset> offsets;
  std::vector<Edge> inEdges;
  std::vector<Edge> outEdges;
  std::vector<size_t> level;
  std::unordered_map<EdgeId, Edge> edges;
};

#endif /* GRAPH_H */
