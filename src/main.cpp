#include "dijkstra.hpp"
#include "graph.hpp"
#include <chrono>
#include <fstream>
#include <iostream>

int main(int argc, char* argv[])
{

  if (argc != 2) {
    std::cout << "Not right amount of arguments!" << '\n';
    std::cout << "expected: \"cr PATH\"" << '\n';
    std::cout << "where path points to a graph file" << '\n';
    return 1;
  }
  const size_t N = 256 * 1024;
  char buffer[N];
  std::string graphPath{ argv[1] }; //NOLINT
  std::ifstream graphFile{};
  graphFile.rdbuf()->pubsetbuf((char*)buffer, N);
  graphFile.open(graphPath);

  std::cout << "Reading Graphdata" << '\n';
  auto start = std::chrono::high_resolution_clock::now();
  Graph g = Graph::createFromStream(graphFile);
  auto end = std::chrono::high_resolution_clock::now();

  using ms = std::chrono::milliseconds;
  std::cout << "creating the graph took " << std::chrono::duration_cast<ms>(end - start).count() << "ms" << '\n';
  Dijkstra d = g.createDijkstra();

  while (true) {
    size_t from, to;
    std::cout << "Please insert NodeId of start node: " << '\n';
    std::cin >> from;
    std::cout << "Please insert NodeId of end node: " << '\n';
    std::cin >> to;

    std::cout << "Starting dijkstra" << '\n';
    auto maybeRoute = d.findBestRoute(NodeId{ from }, NodeId{ to }, Config{ LengthConfig{ 1 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } });

    if (maybeRoute.has_value()) {
      auto route = maybeRoute.value();

      std::cout << "Route is " << route.costs.length << "m long" << '\n';
      std::cout << "Route has " << route.costs.height << "m absolute height difference" << '\n';
      std::cout << "Route has " << route.costs.unsuitability << " unsuitability costs" << '\n';

    } else {
      std::cout << "No route from " << from << "to " << to << "found" << '\n';
    }
  }

  return 0;
}
