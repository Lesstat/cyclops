#ifndef GRAPH_H
#define GRAPH_H

#include "namedType.hpp"
#include <cstdlib>
#include <optional>
#include <vector>

using OsmId = NamedType<size_t, struct OsmIdParameter>;
using NodeId = NamedType<size_t, struct OsmIdParameter>;
using Lat = NamedType<double, struct LatParameter>;
using Lng = NamedType<double, struct LngParameter>;
using Height = NamedType<double, struct HeightParameter>;
using Length = NamedType<double, struct LengthParameter>;
using Unsuitability = NamedType<double, struct UnsuitabilityParameter>;

class Dijkstra;

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
};

struct NodeOffset {
  size_t in;
  size_t out;
  NodeOffset()
      : in(0)
      , out(0)
  {
  }
  NodeOffset(size_t in, size_t out)
      : in(in)
      , out(out)
  {
  }
};

typedef std::optional<size_t>
    ReplacedEdge;

class Edge {
  public:
  Edge(OsmId osmId, NodeId source, NodeId dest);
  Edge(OsmId osmId, NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB);
  Edge(const Edge& other) noexcept;
  Edge(Edge&& other) noexcept;
  virtual ~Edge() noexcept;
  Edge& operator=(const Edge& other);
  Edge& operator=(Edge&& other) noexcept;

  OsmId getSourceId() const;
  OsmId getDestId() const;
  size_t getSourcePos() const;
  size_t getDestPos() const;

  void setSourcePos(size_t pos);
  void setDestPos(size_t pos);

  private:
  void swap(Edge& other);

  size_t internalId;
  OsmId osmId;
  NodeId source;
  NodeId destination;
  size_t sourcePos;
  size_t destPos;
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

  friend std::ostream& operator<<(std::ostream&, const Graph&);

  std::vector<NodeOffset> const& getOffsets() const;
  Dijkstra createDijkstra() const;

  private:
  void swap(Graph& other);

  std::vector<Node> nodes;
  std::vector<NodeOffset> offsets;
  std::vector<Edge> inEdges;
  std::vector<Edge> outEdges;
  std::vector<size_t> level;
};

#endif /* GRAPH_H */
