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
#include "grid.hpp"
#include "ndijkstra.hpp"
#include <future>

template <int Dim>
void Graph<Dim>::connectEdgesToNodes(
    const std::vector<Node>& nodes, const std::vector<EdgeId>& edges)
{
  if (nodes.empty()) {
    return;
  }
  inEdges.reserve(edges.size());
  outEdges.reserve(edges.size());
  const auto max_id = std::max_element(
      nodes.begin(), nodes.end(), [](const auto& a, const auto& b) { return a.id() < b.id(); });

  std::vector<NodePos> map(max_id->id() + 1, NodePos { 0 });
  for (size_t i = 0; i < nodes.size(); ++i) {
    map[nodes[i].id()] = NodePos { i };
  }

  std::for_each(begin(edges), end(edges), [&map, this](const auto& id) {
    // auto& e = Edge::getMutEdge(id);
    auto sourcePos = map[Edge::getSourceId(id)];
    auto destPos = map[Edge::getDestId(id)];
    inEdges.push_back(Edge::makeHalfEdge(id, destPos, sourcePos));
    outEdges.push_back(Edge::makeHalfEdge(id, sourcePos, destPos));
    Edge::sourcePos(id, sourcePos);
    Edge::destPos(id, destPos);
  });
}

enum class Pos { source, dest };
template <int Dim>
void sortEdgesByNodePos(std::vector<HalfEdge<Dim>>& edges, const Graph<Dim>& g, Pos p)
{
  using HalfEdge = HalfEdge<Dim>;
  using Edge = Edge<Dim>;

  auto comparator = [&g, &p](const HalfEdge& a, const HalfEdge& b) {
    const auto& a_begin = p == Pos::source ? Edge::sourcePos(a.id) : Edge::destPos(a.id);
    const auto& b_begin = p == Pos::source ? Edge::sourcePos(b.id) : Edge::destPos(b.id);
    if (a_begin == b_begin) {
      auto aLevel = g.getLevelOf(a.end);
      auto bLevel = g.getLevelOf(b.end);
      if (aLevel == bLevel) {
        return a.end < b.end;
      }
      return aLevel > bLevel;
    }
    return a_begin < b_begin;
  };
  std::sort(edges.begin(), edges.end(), comparator);
}

template <int Dim>
void calculateOffsets(
    std::vector<HalfEdge<Dim>>& edges, std::vector<NodeOffset>& offsets, Pos p, const Graph<Dim>& g)
{
  using Edge = Edge<Dim>;

  auto sourcePos = [&edges](size_t j) { return Edge::sourcePos(edges[j].id); };
  auto destPos = [&edges](size_t j) { return Edge::destPos(edges[j].id); };
  auto setOut = [&offsets](size_t i, size_t j) { offsets[i].out = j; };
  auto setIn = [&offsets](size_t i, size_t j) { offsets[i].in = j; };

  auto getPos
      = [p, sourcePos, destPos](size_t j) { return p == Pos::source ? sourcePos(j) : destPos(j); };

  auto setOffset
      = [p, setOut, setIn](size_t i, size_t j) { p == Pos::source ? setOut(i, j) : setIn(i, j); };

  sortEdgesByNodePos(edges, g, p);

  size_t lastNode = 0;

  for (size_t i = 0; i < edges.size(); ++i) {
    NodePos curNode = getPos(i);
    for (size_t j = lastNode + 1; j < curNode + 1; ++j) {
      setOffset(j, i);
    }
    lastNode = curNode;
  }

  for (size_t i = lastNode + 1; i < offsets.size(); ++i) {
    setOffset(i, edges.size());
  }
}
template <int Dim>
Graph<Dim>::Graph(std::vector<Node>&& nodes, std::vector<Edge>&& edges)
    : edgeCount(edges.size())
{
  auto ids = Edge::administerEdges(std::move(edges));
  init(std::move(nodes), std::move(ids));
}
template <int Dim>
Graph<Dim>::Graph(std::vector<Node>&& nodes, std::vector<EdgeId>&& edges)
    : edgeCount(edges.size())
{
  init(std::move(nodes), std::move(edges));
}
template <int Dim> void Graph<Dim>::init(std::vector<Node>&& nodes, std::vector<EdgeId>&& edges)
{
  std::stable_sort(nodes.begin(), nodes.end(),
      [](const Node& a, const Node& b) { return a.getLevel() < b.getLevel(); });

  connectEdgesToNodes(nodes, edges);
  level.reserve(nodes.size());
  for (const auto& node : nodes) {
    level.push_back(node.getLevel());
  }
  this->nodes = std::move(nodes);
  offsets.reserve(this->nodes.size() + 1);
  for (size_t i = 0; i < this->nodes.size() + 1; ++i) {
    offsets.emplace_back(NodeOffset {});
  }

  auto fut = std::async(
      std::launch::async, [&]() { calculateOffsets(outEdges, offsets, Pos::source, *this); });
  calculateOffsets(inEdges, offsets, Pos::dest, *this);
  fut.wait();
}

template <int Dim> std::vector<NodeOffset> const& Graph<Dim>::getOffsets() const { return offsets; }

template <int Dim> Dijkstra<Dim> Graph<Dim>::createDijkstra()
{
  return Dijkstra { this, nodes.size() };
}

template <int Dim> NormalDijkstra<Dim> Graph<Dim>::createNormalDijkstra(bool unpack)
{
  return NormalDijkstra { this, nodes.size(), unpack };
}

template <int Dim> Grid Graph<Dim>::createGrid(long sideLength) const
{
  Grid b { nodes, sideLength };

  return b;
}

template <int Dim> size_t Graph<Dim>::readCount(std::istream& file)
{
  std::string line;
  std::getline(file, line);
  return std::stoi(line);
}

template <class Obj> void parseLines(std::vector<Obj>& v, std::istream& file, size_t count)
{
  for (size_t i = 0; i < count; ++i) {
    v.push_back(Obj::createFromText(file));
  }
}

template <int Dim> Graph<Dim> Graph<Dim>::createFromStream(std::istream& file)
{
  std::vector<Node> nodes {};
  std::vector<Edge> edges {};
  std::string line {};

  std::getline(file, line);
  while (line.front() == '#') {
    std::getline(file, line);
  }

  size_t dim = Graph::readCount(file);
  if (dim != Dim) {
    std::cerr << "Graph file is of dimension " << dim << " parameters suggests dimension " << Dim
              << '\n';
    throw std::runtime_error("Graph has wrong dimension");
  }
  size_t nodeCount = Graph::readCount(file);
  nodes.reserve(nodeCount);

  size_t edgeCount = Graph::readCount(file);
  edges.reserve(edgeCount);

  parseLines(nodes, file, nodeCount);
  parseLines(edges, file, edgeCount);

  return Graph { std::move(nodes), std::move(edges) };
}

template <int Dim> const Node& Graph<Dim>::getNode(NodePos pos) const { return nodes[pos]; }

template <int Dim> EdgeRange<Dim> Graph<Dim>::getOutgoingEdgesOf(NodePos pos) const
{
  auto start = outEdges.begin();
  std::advance(start, offsets[pos].out);
  auto end = outEdges.begin();
  std::advance(end, offsets[pos + 1].out);
  return EdgeRange { start, end };
}

template <int Dim> EdgeRange<Dim> Graph<Dim>::getIngoingEdgesOf(NodePos pos) const
{
  auto start = inEdges.begin();
  std::advance(start, offsets[pos].in);
  auto end = inEdges.begin();
  std::advance(end, offsets[pos + 1].in);
  return EdgeRange { start, end };
}

template <int Dim> size_t Graph<Dim>::getLevelOf(NodePos pos) const { return level[pos]; }

template <int Dim> std::optional<NodePos> Graph<Dim>::nodePosById(NodeId id) const
{
  for (size_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].id() == id) {
      return NodePos { i };
    }
  }
  return {};
}

template <int Dim> size_t Graph<Dim>::getNodeCount() const { return nodes.size(); }

template <int Dim> size_t Graph<Dim>::getEdgeCount() const { return edgeCount; }

template <int Dim> Graph<Dim> Graph<Dim>::createFromBinaryFile(boost::archive::binary_iarchive& bin)
{
  Graph g { std::vector<Node>(), std::vector<Edge>() };
  bin >> g;
  return g;
}

template <int Dim> size_t Graph<Dim>::getInTimesOutDegree(NodePos node) const
{
  auto& offset = offsets[node];
  return offset.in * offset.out;
}

template <int Dim>
std::unordered_map<NodeId, const Node*> Graph<Dim>::getNodePosByIds(
    const std::unordered_set<NodeId>& ids) const
{
  std::unordered_map<NodeId, const Node*> result;
  result.reserve(ids.size());
  for (const auto& node : nodes) {
    if (ids.find(node.id()) != ids.end()) {
      result[node.id()] = &node;
    }
  }
  return result;
}

template <int Dim> NodePos Graph<Dim>::getNodePos(const Node* n) const
{
  return NodePos { static_cast<size_t>(n - nodes.data()) };
}

template <int Dim> size_t Graph<Dim>::get_max_level()
{
  return *std::max_element(level.begin(), level.end());
}

template <int Dim>
void printRoutes(std::ofstream& dotFile, const Graph<Dim>& graph, const RouteWithCount<Dim>& route1,
    const Route<Dim>& route2, const Config<Dim>& config, const std::set<NodePos>& set)
{
  using Edge = Edge<Dim>;
  using HalfEdge = HalfEdge<Dim>;

  dotFile << "digraph G{" << '\n';
  dotFile << "rankdir=LR;" << '\n';
  dotFile << "size=8;" << '\n';

  auto from = Edge::sourcePos(route1.edges.front());
  auto to = Edge::destPos(route1.edges.back());
  dotFile << "node[ shape = doublecircle color = red]; ";
  printNode(dotFile, graph, from);
  dotFile << " ";
  printNode(dotFile, graph, to);
  dotFile << ";" << '\n';

  dotFile << "node[ shape = circle color = black]; " << '\n';

  std::set<NodePos> printedNodes;

  std::set<EdgeId> printedEdges;
  std::set<EdgeId> route1Edges;
  std::copy(
      route1.edges.begin(), route1.edges.end(), std::inserter(route1Edges, route1Edges.begin()));

  std::set<EdgeId> route2Edges;
  std::transform(route2.edges.begin(), route2.edges.end(),
      std::inserter(route2Edges, route2Edges.begin()), [](const auto& edge) { return edge; });

  for (auto& routeEdgeId : route1.edges) {
    auto node = Edge::sourcePos(routeEdgeId);
    for (auto& edge : graph.getOutgoingEdgesOf(node)) {
      if (printedEdges.count(edge.id) == 0) {
        printedEdges.insert(edge.id);
        printedNodes.insert(edge.end);
        printEdge(dotFile, edge, route1Edges, route2Edges, config, graph);
      }
      // for (auto& edge2 : graph.getOutgoingEdgesOf(edge.end)) {
      //   if (printedEdges.count(edge2.id) == 0) {
      //     printedEdges.insert(edge2.id);
      //     printedNodes.insert(edge2.end);
      //     printEdge(dotFile, edge2, route1Edges, route2Edges, config, graph);
      //   }
      // }
    }
  }

  for (auto& edge : graph.getIngoingEdgesOf(to)) {
    if (printedEdges.count(edge.id) == 0) {
      printedEdges.insert(edge.id);
      printedNodes.insert(edge.end);
      HalfEdge e;
      e.id = edge.id;
      // e.begin = edge.end;
      // e.end = edge.begin;
      e.cost = edge.cost;
      printEdge(dotFile, e, route1Edges, route2Edges, config, graph);
    }
  }

  for (auto& routeEdge : route2.edges) {
    for (auto& edge : graph.getOutgoingEdgesOf(Edge::sourcePos(routeEdge))) {
      if (printedEdges.count(edge.id) == 0) {
        printedEdges.insert(edge.id);
        printedNodes.insert(edge.end);
        printEdge(dotFile, edge, route1Edges, route2Edges, config, graph);
      }
      // for (auto& edge2 : graph.getOutgoingEdgesOf(edge.end)) {
      //   if (printedEdges.count(edge2.id) == 0) {
      //     printedEdges.insert(edge2.id);
      //     printedNodes.insert(edge2.end);
      //     printEdge(dotFile, edge2, route1Edges, route2Edges, config, graph);
      //   }
      // }
    }
  }

  for (auto& node : printedNodes) {
    if (set.count(node) > 0) {
      printNode(dotFile, graph, node);
      dotFile << " [ shape = circle color = blue];\n";
    }
  }
  dotFile << "}" << '\n';
  dotFile.flush();
}

template <int Dim>
void printNode(std::ofstream& dotFile, const Graph<Dim>& graph, const NodePos& pos)
{
  dotFile << '"' << pos << "(" << graph.getLevelOf(pos) << ")" << '"';
}

template <int Dim>
void printEdge(std::ofstream& dotFile, const HalfEdge<Dim>& edge,
    const std::set<EdgeId>& route1Edges, const std::set<EdgeId>& route2Edges,
    const Config<Dim>& config, const Graph<Dim>& g)
{
  using Edge = Edge<Dim>;

  std::string color;
  bool partOfShortcut = route2Edges.count(edge.id) > 0;
  bool partOfRoute = route1Edges.count(edge.id) > 0;
  bool isShortcut = Edge::getEdgeA(edge.id).has_value();
  if (partOfRoute && partOfShortcut) {
    color = "green";
  } else if (partOfShortcut) {
    color = "blue";
  } else if (partOfRoute) {
    color = "yellow";
  } else if (isShortcut) {
    color = "purple";
  } else {
    color = "black";
  }
  // printNode(dotFile, g, edge.begin);
  dotFile << " -> ";
  printNode(dotFile, g, edge.end);
  dotFile << " [label = \"";
  bool first = true;
  for (auto& c : edge.cost.values) {
    if (!first)
      dotFile << ", ";
    dotFile << c;
    first = false;
  }
  dotFile << " | " << edge.cost * config << "\"";
  dotFile << ",color=" << color;
  dotFile << ",labelfontcolor=" << color;
  dotFile << " ]" << '\n';
}
