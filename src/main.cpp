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
#include "server_http.hpp"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iostream>

Graph loadGraphFromTextFile(std::string& graphPath)
{
  const size_t N = 256 * 1024;
  char buffer[N];
  std::ifstream graphFile{};
  graphFile.rdbuf()->pubsetbuf((char*)buffer, N);
  graphFile.open(graphPath);

  std::cout << "Reading Graphdata" << '\n';
  auto start = std::chrono::high_resolution_clock::now();
  Graph g = Graph::createFromStream(graphFile);
  auto end = std::chrono::high_resolution_clock::now();

  using ms = std::chrono::milliseconds;
  std::cout << "creating the graph took " << std::chrono::duration_cast<ms>(end - start).count()
            << "ms" << '\n';
  return g;
}

Graph loadGraphFromBinaryFile(std::string& graphPath)
{
  std::ifstream binFile{ graphPath };
  boost::archive::binary_iarchive bin{ binFile };

  std::cout << "Reading Graphdata" << '\n';
  auto start = std::chrono::high_resolution_clock::now();
  Graph g = Graph::createFromBinaryFile(bin);
  auto end = std::chrono::high_resolution_clock::now();

  using ms = std::chrono::milliseconds;
  std::cout << "creating the graph took " << std::chrono::duration_cast<ms>(end - start).count()
            << "ms" << '\n';
  return g;
}

Graph contractGraph(Graph& g)
{
  Contractor c{};
  auto start = std::chrono::high_resolution_clock::now();
  Graph ch = c.contractCompletely(g);
  auto end = std::chrono::high_resolution_clock::now();
  using m = std::chrono::minutes;
  std::cout << "contracting the graph took " << std::chrono::duration_cast<m>(end - start).count()
            << " minutes" << '\n';
  return ch;
}

void saveToBinaryFile(Graph& ch, std::string& filename)
{
  {
    std::ofstream ofs(filename, std::ios::binary);
    boost::archive::binary_oarchive oa{ ofs };
    oa << ch;
  }
}

void cliDijkstra(Graph& ch)
{
  Dijkstra d = ch.createDijkstra();

  while (true) {
    size_t from, to;
    std::cout << "Please insert NodeId of start node: " << '\n';
    std::cin >> from;
    std::cout << "Please insert NodeId of end node: " << '\n';
    std::cin >> to;

    std::cout << "Starting dijkstra" << '\n';
    auto maybeRoute = d.findBestRoute(NodePos{ from }, NodePos{ to },
        Config{ LengthConfig{ 1 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } });

    if (maybeRoute.has_value()) {
      auto route = maybeRoute.value();

      std::cout << "Route is " << route.costs.length << "m long" << '\n';
      std::cout << "Route has " << route.costs.height << "m absolute height difference" << '\n';
      std::cout << "Route has " << route.costs.unsuitability << " unsuitability costs" << '\n';

    } else {
      std::cout << "No route from " << from << "to " << to << "found" << '\n';
    }
  }
}

void runWebServer()
{
  using HttpServer = SimpleWeb::Server<SimpleWeb::HTTP>;
  using Response = std::shared_ptr<HttpServer::Response>;
  using Request = std::shared_ptr<HttpServer::Request>;

  HttpServer server;
  server.config.port = 8080;

  server.default_resource["GET"] = [](Response response, Request /*request*/) {
    response->write(SimpleWeb::StatusCode::client_error_not_found, "No matching handler found");
  };

  server.resource["^/web/?.*"]["GET"] = [](Response response, Request request) {
    auto web_root_path = boost::filesystem::canonical("web");

    std::string pathWithoutWeb{};
    if (request->path.length() > 4) {
      pathWithoutWeb = request->path.substr(4);
    }

    auto path = boost::filesystem::canonical(web_root_path / pathWithoutWeb);
    // Check if path is within web_root_path
    if (std::distance(web_root_path.begin(), web_root_path.end())
            > std::distance(path.begin(), path.end())
        || !std::equal(web_root_path.begin(), web_root_path.end(), path.begin())) {
      throw std::invalid_argument("path must be within root path");
    }
    if (boost::filesystem::is_directory(path)) {
      path /= "index.html";
    }

    std::ifstream ifs{};
    ifs.open(path.string(), std::ifstream::in | std::ios::binary | std::ios::ate);

    if (!ifs) {
      response->write(SimpleWeb::StatusCode::client_error_not_found, "No such file");
    }
    auto length = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::string buffer(length, '\0');
    ifs.read(&buffer[0], length);
    response->write(buffer);

  };

  std::cout << "Starting web server at http://localhost:" << server.config.port << '\n';
  server.start();
}

namespace po = boost::program_options;
int main(int argc, char* argv[])
{

  std::string textFileName{};
  std::string binFileName{};
  std::string saveFileName{};

  po::options_description loading{ "loading options" };
  loading.add_options()(
      "text,t", po::value<std::string>(&textFileName), "load graph from text file")(
      "bin,b", po::value<std::string>(&binFileName), "load graph form binary file");

  po::options_description action{ "actions" };
  action.add_options()("contract,c", "contract graph")("dijkstra,d",
      "start interactive dijkstra in cli")("web,w", "start webserver for interaction via browser")(
      "save", po::value<std::string>(&saveFileName), "save graph to binary file");

  po::options_description all;
  all.add_options()("help,h", "prints help message");
  all.add(loading).add(action);

  po::variables_map vm{};
  po::store(po::parse_command_line(argc, argv, all), vm);
  po::notify(vm);

  if (vm.count("help") > 0) {
    std::cout << all << '\n';
    return 0;
  }
  Graph g{ std::vector<Node>(), std::vector<Edge>() };
  if (!textFileName.empty()) {
    g = loadGraphFromTextFile(textFileName);
  } else if (!binFileName.empty()) {
    g = loadGraphFromBinaryFile(binFileName);
  } else {
    std::cout << "No input file given" << '\n';
    std::cout << all << '\n';
    return 0;
  }

  if (vm.count("contract") > 0) {
    g = contractGraph(g);
  }

  if (!saveFileName.empty()) {
    saveToBinaryFile(g, saveFileName);
  }

  if (vm.count("dijkstra") > 0) {
    cliDijkstra(g);
  }

  if (vm.count("web") > 0) {
    runWebServer();
  }

  return 0;
}
