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
#include "contractor.hpp"
#include "dijkstra.hpp"
#include "graph.hpp"
#include <boost/archive/binary_oarchive.hpp>
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

  Contractor c{};
  start = std::chrono::high_resolution_clock::now();
  Graph ch = c.contractCompletely(g);
  end = std::chrono::high_resolution_clock::now();
  std::cout << "contracting the graph took " << std::chrono::duration_cast<ms>(end - start).count() << "ms" << '\n';

  {
    std::ofstream ofs("mygraph.ch", std::ios::binary);
    boost::archive::binary_oarchive oa{ ofs };
    oa << ch;
  }

  Dijkstra d = ch.createDijkstra();

  while (true) {
    size_t from, to;
    std::cout << "Please insert NodeId of start node: " << '\n';
    std::cin >> from;
    std::cout << "Please insert NodeId of end node: " << '\n';
    std::cin >> to;

    std::cout << "Starting dijkstra" << '\n';
    auto maybeRoute = d.findBestRoute(NodePos{ from }, NodePos{ to }, Config{ LengthConfig{ 1 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } });

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
