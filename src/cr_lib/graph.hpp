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
#include <fstream>
#include <optional>
#include <set>
#include <unordered_set>
#include <vector>

using NodeId = NamedType<uint32_t, struct NodeIdParameter>;
using NodePos = NamedType<uint32_t, struct NodePosParameter>;
using EdgeId = NamedType<uint32_t, struct EdgeParameter>;
using Lat = NamedType<double, struct LatParameter>;
using Lng = NamedType<double, struct LngParameter>;
using Height = NamedType<double, struct HeightParameter>;
using Length = NamedType<double, struct LengthParameter>;
using Unsuitability = NamedType<double, struct UnsuitabilityParameter>;

template <int Dim> class Dijkstra;
template <int Dim> class NormalDijkstra;
template <int Dim> struct Config;

template <int Dim> struct Cost {
  using ConfigD = Config<Dim>;
  using CostD = Cost<Dim>;

  std::array<double, Dim> values;
  Cost(const std::vector<double>& values)
  {
    for (size_t i = 0; i < Dim; ++i) {
      this->values[i] = values[i];
      if (std::abs(this->values[i]) < 0.0001) {
        this->values[i] = 0;
      }
    }
  }

  Cost()
  {
    for (size_t i = 0; i < Dim; ++i) {
      values[i] = 0;
    }
  }

  Cost(const double values[Dim])
  {
    for (size_t i = 0; i < Dim; ++i) {
      this->values[i] = values[i];
      if (std::abs(this->values[i]) < 0.0001) {
        this->values[i] = 0;
      }
    }
  }
  double operator*(const ConfigD& conf) const;

  CostD operator+(const CostD& c) const
  {
    double newValues[Dim];
    for (size_t i = 0; i < Dim; ++i) {
      newValues[i] = values[i] + c.values[i];
    }
    return newValues;
  };
  CostD operator-(const CostD& c) const
  {
    double newValues[Dim];
    for (size_t i = 0; i < Dim; ++i) {
      newValues[i] = values[i] - c.values[i];
    }
    return newValues;
  };

  bool operator==(const CostD& c) const
  {
    for (size_t i = 0; i < Dim; ++i) {
      if (std::abs(values[i] - c.values[i]) >= 0.0001) {
        return false;
      }
    }
    return true;
  };
  bool operator!=(const CostD& c) const { return !(*this == c); }

  private:
  friend class boost::serialization::access;
  template <class Archive> void serialize(Archive& ar, const unsigned int /*version*/)
  {
    for (auto& val : values)
      ar& val;
  }
};

template <int Dim> struct HalfEdge {
  using CostD = Cost<Dim>;
  using ConfigD = Config<Dim>;

  EdgeId id;
  NodePos end;
  CostD cost;

  double costByConfiguration(const ConfigD& conf) const;

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
  uint32_t in { 0 };
  uint32_t out { 0 };
  NodeOffset() = default;
  NodeOffset(uint32_t in, uint32_t out)
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

template <int Dim> class Edge {
  public:
  using CostD = Cost<Dim>;
  using ConfigD = Config<Dim>;
  using HalfEdgeD = HalfEdge<Dim>;

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

  static NodeId getSourceId(EdgeId i);
  static NodeId getDestId(EdgeId i);
  const ReplacedEdge& getEdgeA() const;
  const ReplacedEdge& getEdgeB() const;

  static const ReplacedEdge& getEdgeA(EdgeId i);
  static const ReplacedEdge& getEdgeB(EdgeId i);

  NodePos sourcePos() const;
  NodePos destPos() const;
  void sourcePos(NodePos source);
  void destPos(NodePos dest);
  static NodePos sourcePos(EdgeId i);
  static NodePos destPos(EdgeId i);
  static void sourcePos(EdgeId i, NodePos source);
  static void destPos(EdgeId i, NodePos dest);

  EdgeId getId() const;
  void setId(EdgeId id);
  const CostD& getCost() const;
  static const CostD& getCost(EdgeId id);
  double costByConfiguration(const ConfigD& conf) const;
  void setCost(CostD c);

  static HalfEdgeD makeHalfEdge(EdgeId id, NodePos begin, NodePos end);

  static Edge createFromText(std::istream& text);
  static std::vector<EdgeId> administerEdges(std::vector<Edge>&& edges);
  static const Edge getEdge(EdgeId id);

  template <int D>
  friend void testEdgeInternals(const Edge<D>& e, NodeId source, NodeId destination, Length length,
      Height height, Unsuitability unsuitability, const ReplacedEdge& edgeA,
      const ReplacedEdge& edgeB);

  private:
  friend class boost::serialization::access;

  EdgeId internalId;
  NodeId source;
  NodeId destination;
  CostD cost;
  ReplacedEdge edgeA;
  ReplacedEdge edgeB;
  NodePos sourcePos_;
  NodePos destPos_;

  static std::vector<EdgeId> internalId_vec;
  static std::vector<NodeId> source_vec;
  static std::vector<NodeId> destination_vec;
  static std::vector<CostD> cost_vec;
  static std::vector<ReplacedEdge> edgeA_vec;
  static std::vector<ReplacedEdge> edgeB_vec;
  static std::vector<NodePos> sourcePos__vec;
  static std::vector<NodePos> destPos__vec;

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
  Node(NodeId id, Lat lat, Lng lng, double height);
  Node(const Node& other) = default;
  Node(Node&& other) noexcept = default;
  virtual ~Node() noexcept = default;
  Node& operator=(const Node& other) = default;
  Node& operator=(Node&& other) noexcept = default;

  void assignLevel(uint32_t level);
  size_t getLevel() const;
  NodeId id() const;
  friend std::ostream& operator<<(std::ostream& os, const Node& n);

  static Node createFromText(std::istream& text);
  friend void testNodeInternals(const Node& n, NodeId id, Lat lat, Lng lng, size_t level);

  Lat lat() const;
  Lng lng() const;
  short height() const;

  private:
  friend class boost::serialization::access;

  NodeId id_;
  Lat lat_;
  Lng lng_;
  uint32_t level = 0;
  float height_;
  template <class Archive> void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& id_;
    ar& lat_;
    ar& lng_;
    ar& level;
    ar& height_;
  }
};

class Grid;
template <int Dim> class EdgeRange;
template <int Dim> class Graph {
  public:
  using EdgeD = Edge<Dim>;
  using HalfEdgeD = HalfEdge<Dim>;
  using CostD = Cost<Dim>;
  using EdgeRangeD = EdgeRange<Dim>;
  using DijkstraD = Dijkstra<Dim>;
  using NormalDijkstraD = NormalDijkstra<Dim>;

  Graph(std::vector<Node>&& nodes, std::vector<EdgeD>&& edges);
  Graph(std::vector<Node>&& nodes, std::vector<EdgeId>&& edges);
  Graph(const Graph& other) = delete;
  Graph(Graph&& other) noexcept = default;
  virtual ~Graph() noexcept = default;
  Graph& operator=(const Graph& other) = delete;
  Graph& operator=(Graph&& other) noexcept = default;

  template <int D> friend std::ostream& operator<<(std::ostream& /*s*/, const Graph<D>& /*g*/);

  std::vector<NodeOffset> const& getOffsets() const;
  DijkstraD createDijkstra();
  NormalDijkstraD createNormalDijkstra(bool unpack = false);
  Grid createGrid(uint32_t sideLength = 100) const;

  EdgeRangeD getOutgoingEdgesOf(NodePos pos) const;
  EdgeRangeD getIngoingEdgesOf(NodePos pos) const;

  size_t getLevelOf(NodePos pos) const;

  static Graph createFromStream(std::istream& file);
  static Graph createFromBinaryFile(boost::archive::binary_iarchive& bin);

  const Node& getNode(NodePos pos) const;
  std::optional<NodePos> nodePosById(NodeId id) const;

  uint32_t getNodeCount() const;
  uint32_t getEdgeCount() const;

  uint32_t getInTimesOutDegree(NodePos node) const;

  std::unordered_map<NodeId, const Node*> getNodePosByIds(
      const std::unordered_set<NodeId>& ids) const;

  NodePos getNodePos(const Node* n) const;
  uint32_t get_max_level();

  private:
  friend class boost::serialization::access;

  void init(std::vector<Node>&& nodes, std::vector<EdgeId>&& edges);
  void connectEdgesToNodes(const std::vector<Node>& nodes, const std::vector<EdgeId>& edges);

  static size_t readCount(std::istream& file);

  std::vector<Node> nodes;
  std::vector<NodeOffset> offsets;
  std::vector<HalfEdgeD> inEdges;
  std::vector<HalfEdgeD> outEdges;
  std::vector<uint32_t> level;
  size_t edgeCount;

  template <class Archive> void save(Archive& ar, const unsigned int /*version*/) const
  {
    ar& nodes;
    std::vector<EdgeD> edges {};
    edges.reserve(edgeCount);
    for (const auto& e : inEdges) {
      edges.push_back(EdgeD::getEdge(e.id));
    }
    std::sort(edges.begin(), edges.end(),
        [](const auto& left, const auto& right) { return left.getId() < right.getId(); });
    ar& edges;
  }

  template <class Archive> void load(Archive& ar, const unsigned int /*version*/)
  {
    std::vector<Node> nodes;
    std::vector<EdgeD> edges;
    ar& nodes;
    ar& edges;
    std::sort(edges.begin(), edges.end(),
        [](const auto& left, const auto& right) { return left.getId() < right.getId(); });
    *this = Graph(std::move(nodes), std::move(edges));
  }
  BOOST_SERIALIZATION_SPLIT_MEMBER()
};

template <int Dim> class EdgeRange {
  public:
  using HalfEdgeD = HalfEdge<Dim>;
  using iterator = typename std::vector<HalfEdgeD>::const_iterator;

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

template <int Dim> struct RouteWithCount;
template <int Dim> struct Route;

template <int Dim>
void printRoutes(std::ofstream& dotFile, const Graph<Dim>& graph, const RouteWithCount<Dim>& route1,
    const Route<Dim>& route2, const Config<Dim>& config, const std::set<NodePos>& set = {});

#include "edge.inc"
#include "graph.inc"

#endif /* GRAPH_H */
