#include "graph.hpp"
#include <iostream>
#include <vector>

int main()
{

  std::vector<Node> nodes;
  nodes.push_back(Node{ OsmId(1), Lat(3.4), Lng(4.6), Height(3.4) });
  nodes.push_back(Node{ OsmId(2), Lat(3.4), Lng(4.6), Height(3.4) });
  nodes.push_back(Node{ OsmId(3), Lat(3.4), Lng(4.6), Height(3.4) });
  nodes.push_back(Node{ OsmId(4), Lat(3.4), Lng(4.6), Height(3.4) });

  std::vector<Edge> edges;
  edges.push_back(Edge{ OsmId(7), NodeId(2), NodeId(1) });
  edges.push_back(Edge{ OsmId(6), NodeId(2), NodeId(2) });
  edges.push_back(Edge{ OsmId(4), NodeId(1), NodeId(3) });
  edges.push_back(Edge{ OsmId(5), NodeId(1), NodeId(3) });

  Graph g{ std::move(nodes), std::move(edges) };

  std::cout << "my graph: " << g << "\n";

  return 0;
}
