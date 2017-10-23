#include "graph.hpp"
#include <iostream>
#include <unordered_map>

void connectEdgesToNodes(const std::vector<Node>& nodes, std::vector<Edge>& edges)
{
  std::unordered_map<size_t, size_t> map;
  map.reserve(nodes.size());
  for (size_t i = 0; i < nodes.size(); i++) {
    map.insert({ nodes[i].getOsmId().get(), i });
  }

  std::for_each(begin(edges), end(edges), [&map](Edge& e) {
    size_t sourcePos = map[e.getSourceId().get()];
    e.setSourcePos(sourcePos);
    size_t destPos = map[e.getDestId().get()];
    e.setDestPos(destPos);
  });
}

enum class Pos { source,
  dest };
void sortEdgesByNodePos(std::vector<Edge>& edges, Pos p)
{
  auto sourceSort = [](const Edge& a, const Edge& b) { return a.getSourcePos() < b.getSourcePos(); };
  auto destSort = [](const Edge& a, const Edge& b) { return a.getDestPos() < b.getDestPos(); };

  if (p == Pos::source) {
    std::sort(edges.begin(), edges.end(), sourceSort);
  } else {
    std::sort(edges.begin(), edges.end(), destSort);
  }
}

void calculateOffsets(std::vector<Edge>& edges, std::vector<NodeOffset>& offsets, Pos p)
{
  size_t lastOffset = 0;
  auto sourcePos = [&edges](size_t j) { return edges[j].getSourcePos(); };
  auto destPos = [&edges](size_t j) { return edges[j].getDestPos(); };
  auto setOut = [&offsets](size_t i, size_t j) { offsets[i].out = j; };
  auto setIn = [&offsets](size_t i, size_t j) { offsets[i].in = j; };

  auto getPos = [p, sourcePos, destPos](size_t j) {
    return p == Pos::source ? sourcePos(j) : destPos(j);
  };

  auto setOffset = [p, setOut, setIn, &lastOffset](size_t i, size_t j) {
    lastOffset = j + 1;
    p == Pos::source ? setOut(i, lastOffset) : setIn(i, lastOffset);
  };

  sortEdgesByNodePos(edges, p);

  size_t index = offsets.size() - 1;
  size_t value = edges.size() - 1;
  setOffset(index, value);
  size_t j = edges.size();

  for (size_t i = offsets.size() - 1; i-- > 0;) {
    for (; j-- > 0;) {
      size_t pos = getPos(j);
      if (pos < i) {
        setOffset(i, j);
        j++;
        break;
      }
    }
  }
}

Graph::Graph(std::vector<Node>&& nodes, std::vector<Edge>&& edges)
{
  connectEdgesToNodes(nodes, edges);
  this->nodes = std::move(nodes);
  offsets.reserve(this->nodes.size() + 1);
  for (size_t i = 0; i < this->nodes.size() + 1; ++i)
    offsets.push_back(NodeOffset{});

  calculateOffsets(edges, offsets, Pos::source);

  calculateOffsets(edges, offsets, Pos::dest);
}

Graph::Graph(const Graph& other)
    : nodes(other.nodes)
    , offsets(other.offsets)
    , inEdges(other.inEdges)
    , outEdges(other.outEdges)
    , level(other.level)
{
  std::cout << "Graph is copied. Did you really want that?"
            << '\n';
}

Graph::Graph(Graph&& other) noexcept
{
  swap(other);
}

Graph::~Graph() noexcept {}
Graph& Graph::operator=(const Graph& other)
{
  Graph tmp{ other };
  swap(tmp);
  return *this;
}
Graph& Graph::operator=(Graph&& other) noexcept
{
  swap(other);
  return *this;
}

void Graph::swap(Graph& other)
{
  std::swap(nodes, other.nodes);
  std::swap(offsets, other.offsets);
  std::swap(inEdges, other.inEdges);
  std::swap(outEdges, other.outEdges);
  std::swap(level, other.level);
}

std::ostream& operator<<(std::ostream& s, const Graph& g)
{

  s << "Node positions" << '\n';
  for (auto node : g.nodes) {
    s << node << ", ";
  }
  s << '\n';
  s << "offsets for out" << '\n';
  for (auto off : g.offsets) {
    s << off.out << ", ";
  }
  s << '\n';
  s << "offsets for in" << '\n';
  for (auto off : g.offsets) {
    s << off.in << ", ";
  }
  return s;
}
