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
#include <cstdlib>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

using OsmId = NamedType<size_t, struct OsmIdParameter>;
using NodeId = NamedType<size_t, struct NodeIdParameter>;
using EdgeId = NamedType<size_t, struct EdgeParameter>;
using Lat = NamedType<double, struct LatParameter>;
using Lng = NamedType<double, struct LngParameter>;
using Height = NamedType<short, struct HeightParameter>;
using Length = NamedType<double, struct LengthParameter>;
using Unsuitability = NamedType<short, struct UnsuitabilityParameter>;

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

  Cost operator+(const Cost& c) const
  {
    return Cost{ Length(length.get() + c.length.get()), Height(height.get() + c.height.get()), Unsuitability(unsuitability.get() + c.unsuitability.get()) };
  };
  Cost operator-(const Cost& c) const
  {
    return Cost{ Length(length.get() - c.length.get()), Height(height.get() - c.height.get()), Unsuitability(unsuitability.get() - c.unsuitability.get()) };
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

using ReplacedEdge = std::optional<EdgeId>;

class Edge {
  public:
  Edge(NodeId source, NodeId dest);
  Edge(NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB);
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
  void setCost(Cost c);

  static Edge createFromText(const std::string& text);

  friend void testEdgeInternals(const Edge& e,
      NodeId source,
      NodeId destination,
      Length length,
      Height height,
      Unsuitability unsuitability,
      const ReplacedEdge& edgeA,
      const ReplacedEdge& edgeB);

  private:
  void swap(Edge& other);

  EdgeId internalId;
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
  friend void testNodeInternals(const Node& n, OsmId osmId, Lat lat, Lng lng, Height height, size_t level);

  private:
  void swap(Node& other);

  OsmId osmId;
  Lat lat;
  Lng lng;
  Height height;
  size_t level;
};

class EdgeRange;
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
  Dijkstra createDijkstra();

  EdgeRange getOutgoingEdgesOf(NodeId n) const;
  EdgeRange getIngoingEdgesOf(NodeId n) const;

  size_t getLevelOf(NodeId n) const;
  const Edge& getEdge(EdgeId e) const;

  static Graph createFromStream(std::istream& file);

  const Node& getNode(NodeId id) const;

  size_t getNodeCount() const;

  private:
  void swap(Graph& other);

  std::vector<Node> nodes;
  std::vector<NodeOffset> offsets;
  std::vector<EdgeId> inEdges;
  std::vector<EdgeId> outEdges;
  std::vector<size_t> level;
  std::unordered_map<EdgeId, Edge> edges;
  std::shared_ptr<Graph> self;
};

class EdgeRange {
  public:
  using iterator = std::vector<EdgeId>::const_iterator;

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
