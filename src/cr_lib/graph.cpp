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
#include "graph.hpp"
#include "dijkstra.hpp"
#include <iostream>
#include <iterator>
#include <unordered_map>

void connectEdgesToNodes(const std::vector<Node>& nodes, std::vector<Edge>& edges)
{
  std::unordered_map<NodeId, NodePos> map;
  map.reserve(nodes.size());
  for (size_t i = 0; i < nodes.size(); i++) {
    map.insert({ nodes[i].id(), NodePos{ i } });
  }

  std::for_each(begin(edges), end(edges), [&map](Edge& e) {
    auto sourcePos = map[e.getSourceId()];
    e.setSourcePos(sourcePos);
    auto destPos = map[e.getDestId()];
    e.setDestPos(destPos);
  });
}

enum class Pos {
  source,
  dest
};
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
  auto sourcePos = [&edges](size_t j) { return edges[j].getSourcePos(); };
  auto destPos = [&edges](size_t j) { return edges[j].getDestPos(); };
  auto setOut = [&offsets](size_t i, size_t j) { offsets[i].out = j; };
  auto setIn = [&offsets](size_t i, size_t j) { offsets[i].in = j; };

  auto getPos = [p, sourcePos, destPos](size_t j) {
    return p == Pos::source ? sourcePos(j) : destPos(j);
  };

  auto setOffset = [p, setOut, setIn](size_t i, size_t j) {
    p == Pos::source ? setOut(i, j) : setIn(i, j);
  };

  sortEdgesByNodePos(edges, p);

  size_t lastNode = 0;

  for (size_t i = 0; i < edges.size(); ++i) {
    size_t curNode = getPos(i);
    for (size_t j = lastNode + 1; j < curNode + 1; ++j) {
      setOffset(j, i);
    }
    lastNode = curNode;
  }

  for (size_t i = lastNode + 1; i < offsets.size(); ++i) {
    setOffset(i, edges.size());
  }
}

Graph::Graph(std::vector<Node>&& nodes, std::vector<Edge>&& edges)
{
  std::stable_sort(nodes.begin(),
      nodes.end(),
      [](const Node& a, const Node& b) { return a.getLevel() < b.getLevel(); });

  connectEdgesToNodes(nodes, edges);
  for (const auto& node : nodes) {
    level.push_back(node.getLevel());
  }
  for (const auto& edge : edges) {
    this->edges.insert({ edge.getId(), edge });
  }
  this->nodes = std::move(nodes);
  offsets.reserve(this->nodes.size() + 1);
  for (size_t i = 0; i < this->nodes.size() + 1; ++i) {
    offsets.emplace_back(NodeOffset{});
  }

  calculateOffsets(edges, offsets, Pos::source);
  std::transform(edges.begin(), edges.end(), std::back_inserter(outEdges), [](const Edge& e) { return e.getId(); });

  calculateOffsets(edges, offsets, Pos::dest);
  std::transform(edges.begin(), edges.end(), std::back_inserter(inEdges), [](const Edge& e) { return e.getId(); });
}

std::ostream& operator<<(std::ostream& s, const Graph& g)
{

  s << "Node positions" << '\n';
  for (auto const& node : g.nodes) {
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

std::vector<NodeOffset> const& Graph::getOffsets() const
{
  return offsets;
}

Dijkstra Graph::createDijkstra()
{
  return Dijkstra{ this, nodes.size() };
}

size_t readCount(std::istream& file)
{
  std::string line;
  std::getline(file, line);
  return std::stoi(line);
}

template <class Obj>
void parseLines(std::vector<Obj>& v, std::istream& file, size_t count)
{
  std::string line{};
  for (size_t i = 0; i < count; ++i) {
    std::getline(file, line);
    v.push_back(Obj::createFromText(line));
  }
}

Graph Graph::createFromStream(std::istream& file)
{
  std::vector<Node> nodes{};
  std::vector<Edge> edges{};
  std::string line{};

  std::getline(file, line);
  while (line.front() == '#') {
    std::getline(file, line);
  }
  size_t nodeCount = readCount(file);
  nodes.reserve(nodeCount);

  size_t edgeCount = readCount(file);
  edges.reserve(edgeCount);

  parseLines(nodes, file, nodeCount);
  parseLines(edges, file, edgeCount);

  return Graph{ std::move(nodes), std::move(edges) };
}

const Node& Graph::getNode(NodePos pos) const
{
  return nodes[pos];
}

const Edge& Graph::getEdge(EdgeId e) const
{
  return edges.at(e);
}

EdgeRange Graph::getOutgoingEdgesOf(NodePos pos) const
{
  auto start = outEdges.begin();
  std::advance(start, offsets[pos].out);
  auto end = outEdges.begin();
  std::advance(end, offsets[pos + 1].out);
  return EdgeRange{ start, end };
}

EdgeRange Graph::getIngoingEdgesOf(NodePos pos) const
{
  auto start = inEdges.begin();
  std::advance(start, offsets[pos].in);
  auto end = inEdges.begin();
  std::advance(end, offsets[pos + 1].in);
  return EdgeRange{ start, end };
}

size_t Graph::getLevelOf(NodePos pos) const
{
  return level[pos];
}

std::optional<NodePos> Graph::nodePosById(NodeId id) const
{
  for (size_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].id() == id) {
      return NodePos{ i };
    }
  }
  return {};
}

size_t Graph::getNodeCount() const
{
  return nodes.size();
}

size_t Graph::getEdgeCount() const
{
  return edges.size();
}
