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
#ifndef GRAPH_H
#define GRAPH_H

#include "namedType.hpp"
#include "serialize_optional.hpp"
#include <atomic>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/vector.hpp>
#include <optional>
#include <vector>

using OsmId = NamedType<size_t, struct OsmIdParameter>;
using NodeId = NamedType<size_t, struct NodeIdParameter>;
using NodePos = NamedType<size_t, struct NodePosParameter>;
using EdgeId = NamedType<size_t, struct EdgeParameter>;
using Lat = NamedType<double, struct LatParameter>;
using Lng = NamedType<double, struct LngParameter>;
using Height = NamedType<short, struct HeightParameter>;
using Length = NamedType<double, struct LengthParameter>;
using Unsuitability = NamedType<short, struct UnsuitabilityParameter>;

class Dijkstra;
class NormalDijkstra;
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
  float operator*(const Config& conf) const;

  Cost operator+(const Cost& c) const
  {
    return Cost{ Length(length.get() + c.length.get()), Height(height.get() + c.height.get()),
      Unsuitability(unsuitability.get() + c.unsuitability.get()) };
  };
  Cost operator-(const Cost& c) const
  {
    return Cost{ Length(length.get() - c.length.get()), Height(height.get() - c.height.get()),
      Unsuitability(unsuitability.get() - c.unsuitability.get()) };
  };

  private:
  friend class boost::serialization::access;
  template <class Archive> void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& length;
    ar& height;
    ar& unsuitability;
  }
};

struct HalfEdge {
  EdgeId id;
  NodePos end;
  NodePos begin;
  Cost cost;

  float costByConfiguration(const Config& conf) const;

  private:
  friend class boost::serialization::access;
  template <class Archive> void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& id;
    ar& end;
    ar& cost;
  }
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

  private:
  friend class boost::serialization::access;
  template <class Archive> void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& in;
    ar& out;
  }
};

using ReplacedEdge = std::optional<EdgeId>;

class Edge {
  public:
  Edge() = default;
  Edge(NodeId source, NodeId dest);
  Edge(NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB);
  Edge(const Edge& other) = default;
  Edge(Edge&& other) noexcept = default;
  virtual ~Edge() noexcept = default;
  Edge& operator=(const Edge& other) = default;
  Edge& operator=(Edge&& other) noexcept = default;

  NodeId getSourceId() const;
  NodeId getDestId() const;
  const ReplacedEdge& getEdgeA() const;
  const ReplacedEdge& getEdgeB() const;

  EdgeId getId() const;
  const Cost& getCost() const;
  double costByConfiguration(const Config& conf) const;
  void setCost(Cost c);

  HalfEdge makeHalfEdge(NodePos begin, NodePos end) const;

  static Edge createFromText(const std::string& text);
  static void administerEdges(const std::vector<Edge>& edges);
  static const Edge& getEdge(EdgeId id);
  static void init();

  friend void testEdgeInternals(const Edge& e, NodeId source, NodeId destination, Length length,
      Height height, Unsuitability unsuitability, const ReplacedEdge& edgeA,
      const ReplacedEdge& edgeB);

  private:
  friend class boost::serialization::access;

  EdgeId internalId;
  NodeId source;
  NodeId destination;
  Cost cost;
  ReplacedEdge edgeA;
  ReplacedEdge edgeB;

  static std::atomic<size_t> lastId;
  static std::vector<Edge> edges;

  template <class Archive> void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& internalId;
    ar& source;
    ar& destination;
    ar& cost;
    ar& edgeA;
    ar& edgeB;
  }
};

class Node {
  public:
  Node() = default;
  Node(NodeId id, Lat lat, Lng lng, Height height);
  Node(const Node& other) = default;
  Node(Node&& other) noexcept = default;
  virtual ~Node() noexcept = default;
  Node& operator=(const Node& other) = default;
  Node& operator=(Node&& other) noexcept = default;

  void assignLevel(size_t level);
  size_t getLevel() const;
  NodeId id() const;
  friend std::ostream& operator<<(std::ostream& os, const Node& n);

  static Node createFromText(const std::string& text);
  friend void testNodeInternals(
      const Node& n, NodeId id, Lat lat, Lng lng, Height height, size_t level);

  Lat lat() const;
  Lng lng() const;

  private:
  friend class boost::serialization::access;

  NodeId id_;
  Lat lat_;
  Lng lng_;
  Height height;
  size_t level = 0;
  template <class Archive> void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& id_;
    ar& lat_;
    ar& lng_;
    ar& height;
    ar& level;
  }
};

class Grid;
class EdgeRange;
class Graph {
  public:
  Graph(std::vector<Node>&& nodes, std::vector<Edge>&& edges);
  Graph(std::vector<Node>&& nodes, std::vector<EdgeId>&& edges);
  Graph(const Graph& other) = delete;
  Graph(Graph&& other) noexcept = default;
  virtual ~Graph() noexcept = default;
  Graph& operator=(const Graph& other) = delete;
  Graph& operator=(Graph&& other) noexcept = default;

  friend std::ostream& operator<<(std::ostream& /*s*/, const Graph& /*g*/);

  std::vector<NodeOffset> const& getOffsets() const;
  Dijkstra createDijkstra();
  NormalDijkstra createNormalDijkstra(bool unpack = false);
  Grid createGrid(long sideLength = 100) const;

  EdgeRange getOutgoingEdgesOf(NodePos pos) const;
  EdgeRange getIngoingEdgesOf(NodePos pos) const;

  size_t getLevelOf(NodePos pos) const;

  static Graph createFromStream(std::istream& file);
  static Graph createFromBinaryFile(boost::archive::binary_iarchive& bin);

  const Node& getNode(NodePos pos) const;
  std::optional<NodePos> nodePosById(NodeId id) const;

  size_t getNodeCount() const;
  size_t getEdgeCount() const;

  private:
  friend class boost::serialization::access;

  void init(std::vector<Node>&& nodes, std::vector<EdgeId>&& edges);
  void connectEdgesToNodes(const std::vector<Node>& nodes, const std::vector<EdgeId>& edges);

  std::vector<Node> nodes;
  std::vector<NodeOffset> offsets;
  std::vector<HalfEdge> inEdges;
  std::vector<HalfEdge> outEdges;
  std::vector<size_t> level;
  size_t edgeCount;

  template <class Archive> void save(Archive& ar, const unsigned int /*version*/) const
  {
    ar& nodes;
    std::vector<Edge> edges{};
    edges.reserve(edgeCount);
    for (const auto& e : inEdges) {
      edges.push_back(Edge::getEdge(e.id));
    }
    ar& edges;
  }
  template <class Archive> void load(Archive& ar, const unsigned int /*version*/)
  {
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    ar& nodes;
    ar& edges;
    *this = Graph(std::move(nodes), std::move(edges));
  }
  BOOST_SERIALIZATION_SPLIT_MEMBER()
};

class EdgeRange {
  public:
  using iterator = std::vector<HalfEdge>::const_iterator;

  EdgeRange(iterator begin, iterator end)
      : begin_(begin)
      , end_(end)
  {
  }
  EdgeRange(const EdgeRange& other) = default;
  EdgeRange(EdgeRange&& other) noexcept = default;
  virtual ~EdgeRange() noexcept = default;
  EdgeRange& operator=(const EdgeRange& other) = default;
  EdgeRange& operator=(EdgeRange&& other) noexcept = default;

  iterator begin() const { return begin_; }
  iterator end() const { return end_; }

  protected:
  private:
  iterator begin_;
  iterator end_;
};

#endif /* GRAPH_H */
