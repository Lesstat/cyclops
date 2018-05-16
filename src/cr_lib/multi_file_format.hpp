
#include "graph.hpp"
#include <boost/filesystem.hpp>
#include <fstream>

Node createNode(std::ifstream& graph, std::ifstream& labels)
{
  size_t id, osmId, level;
  double lat, lng, height;

  graph >> id >> osmId >> lat >> lng >> height >> level;
  labels >> level;

  Node n{ NodeId{ id }, Lat(lat), Lng(lng), Height(height) };
  n.assignLevel(level);
  return n;
}

Edge createEdge(std::ifstream& ch, std::ifstream& skips)
{

  size_t source, dest;
  double length, height, unsuitability;
  long edgeA, edgeB;

  ch >> source >> dest >> length >> height >> unsuitability;
  skips >> edgeA >> edgeB;

  Edge e{ NodeId(source), NodeId(dest) };
  if (edgeA > 0) {
    e.edgeA = EdgeId{ static_cast<size_t>(edgeA) };
    e.edgeB = EdgeId{ static_cast<size_t>(edgeB) };
  }

  Cost cost;
  cost.length = Length(length);
  cost.height = Height(height);
  cost.unsuitability = Unsuitability(unsuitability);

  e.setCost(cost);
  assert(e.cost.length >= 0);
  assert(e.cost.height >= 0);
  assert(e.cost.unsuitability >= 0);
  return e;
}

Graph readMultiFileGraph(std::string graphPath)
{
  using namespace boost::filesystem;

  auto directory = canonical(graphPath).parent_path();

  path chGraph{ directory };
  chGraph /= "ch_graph";

  path nodeLabels{ directory };
  nodeLabels /= "node_labels";

  path skips{ directory };
  skips /= "skips";

  std::ifstream graphFile{};
  graphFile.open(graphPath);

  std::ifstream chFile{};
  chFile.open(chGraph.string());
  std::ifstream nodeLabelsFile{ nodeLabels.string() };
  std::ifstream skipsFile{ skips.string() };

  std::string line;
  std::getline(graphFile, line);
  while (line.front() == '#') {
    std::getline(graphFile, line);
  }
  size_t nodeCountGraph = 0;
  size_t nodeCountCh = 0;

  graphFile >> nodeCountGraph;
  chFile >> nodeCountCh;

  if (nodeCountGraph != nodeCountCh) {
    throw std::invalid_argument("node counts of ch and graph do not match");
  }

  size_t edgeCount = 0;
  graphFile >> edgeCount;
  chFile >> edgeCount;

  std::vector<Node> nodes;
  nodes.reserve(nodeCountGraph);
  std::vector<Edge> edges;
  edges.reserve(edgeCount);

  for (size_t i = 0; i < nodeCountGraph; ++i) {
    nodes.push_back(createNode(graphFile, nodeLabelsFile));
  }

  for (size_t i = 0; i < edgeCount; ++i) {
    edges.push_back(createEdge(chFile, skipsFile));
  }

  return Graph{ std::move(nodes), std::move(edges) };
}
