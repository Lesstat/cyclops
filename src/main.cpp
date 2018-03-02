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
#include "grid.hpp"
#include "routeComparator.hpp"
#include "server_http.hpp"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <random>

Config generateRandomConfig()
{
  std::random_device rd{};
  std::uniform_real_distribution lenDist(0.0, 1.0);
  LengthConfig l(lenDist(rd));
  std::uniform_real_distribution heightDist(0.0, 1.0 - l.get());
  HeightConfig h(heightDist(rd));
  UnsuitabilityConfig u(1 - l - h);

  return Config{ l, h, u };
}

std::string routeToJson(const Route& route, const Graph& g)
{
  std::stringstream resultJson;
  resultJson << "{ \"length\": " << route.costs.length << ", \"height\": " << route.costs.height
             << ", \"unsuitability\": " << route.costs.unsuitability
             << R"(, "route": { "type": "Feature", "geometry": { "type": "LineString", )"
             << "\"coordinates\":[";

  std::unordered_set<NodeId> nodes;
  nodes.reserve(route.edges.size());
  for (const auto& edge : route.edges) {
    nodes.insert(edge.getSourceId());
    nodes.insert(edge.getDestId());
  }
  auto idToNode = g.getNodePosByIds(nodes);

  for (const auto& edge : route.edges) {
    auto node = idToNode[edge.getSourceId()];
    resultJson << '[' << node->lng() << ", " << node->lat() << "],";
  }
  if (!route.edges.empty()) {
    const auto& lastEdge = route.edges[route.edges.size() - 1];
    auto endNode = idToNode[lastEdge.getDestId()];
    resultJson << '[' << endNode->lng() << ", " << endNode->lat() << "]] } } }";
  }
  return resultJson.str();
}

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

Graph contractGraph(Graph& g, unsigned short rest, bool printStats)
{
  Contractor c{ printStats };
  auto start = std::chrono::high_resolution_clock::now();
  Graph ch = c.contractCompletely(g, rest);
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

void runWebServer(Graph& g)
{
  using HttpServer = SimpleWeb::Server<SimpleWeb::HTTP>;
  using Response = std::shared_ptr<HttpServer::Response>;
  using Request = std::shared_ptr<HttpServer::Request>;

  Grid grid = g.createGrid();
  Dijkstra dijkstra = g.createDijkstra();

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
  server.resource["^/node_at"]["GET"] = [&grid](Response response, Request request) {
    const double IMPOSSIBLE_VALUE = -1000;
    double lat = IMPOSSIBLE_VALUE;
    double lng = IMPOSSIBLE_VALUE;

    auto query_fields = request->parse_query_string();
    for (const auto& field : query_fields) {
      if (field.first == "lat") {
        lat = stod(field.second);
      } else if (field.first == "lng") {
        lng = stod(field.second);
      }
    }
    if (lat == IMPOSSIBLE_VALUE || lng == IMPOSSIBLE_VALUE) {
      response->write(SimpleWeb::StatusCode::client_error_bad_request,
          "Request needs to contain lat and lng parameters for query");
    }
    auto pos = grid.findNextNode(Lat{ lat }, Lng{ lng });
    response->write(SimpleWeb::StatusCode::success_ok, std::to_string(pos->get()));

  };

  server.resource["^/route"]["GET"] = [&g, &dijkstra](Response response, Request request) {
    std::optional<size_t> s{}, t{}, length{}, height{}, unsuitability{};
    auto query_fields = request->parse_query_string();
    for (const auto& field : query_fields) {
      if (field.first == "s") {
        s = static_cast<size_t>(stoull(field.second));
      } else if (field.first == "t") {
        t = static_cast<size_t>(stoull(field.second));
      } else if (field.first == "length") {
        length = static_cast<size_t>(stoull(field.second));
      } else if (field.first == "height") {
        height = static_cast<size_t>(stoull(field.second));
      } else if (field.first == "unsuitability") {
        unsuitability = static_cast<size_t>(stoull(field.second));
      }
    }

    if (!(s && t && length && height && unsuitability)) {
      response->write(SimpleWeb::StatusCode::client_error_bad_request,
          "Request needs to contain the parameters: s, t, length, height, unsuitability");
    }

    auto start = std::chrono::high_resolution_clock::now();
    auto route = dijkstra.findBestRoute(NodePos{ *s }, NodePos{ *t },
        Config{ LengthConfig{ *length / 100.0 }, HeightConfig{ *height / 100.0 },
            UnsuitabilityConfig{ *unsuitability / 100.0 } });
    auto end = std::chrono::high_resolution_clock::now();

    using ms = std::chrono::milliseconds;
    std::cout << "Finding the route took " << std::chrono::duration_cast<ms>(end - start).count()
              << "ms" << '\n';
    if (route) {

      auto json = routeToJson(*route, g);
      SimpleWeb::CaseInsensitiveMultimap header;
      header.emplace("Content-Type", "application/json");
      response->write(SimpleWeb::StatusCode::success_ok, json, header);
    }
    response->write(SimpleWeb::StatusCode::client_error_not_found, "Did not find route");

  };

  server.resource["^/alternative/random"]["GET"]
      = [&g, &dijkstra](Response response, Request request) {
          std::optional<size_t> s{}, t{};
          auto query_fields = request->parse_query_string();
          for (const auto& field : query_fields) {
            if (field.first == "s") {
              s = static_cast<size_t>(stoull(field.second));
            } else if (field.first == "t") {
              t = static_cast<size_t>(stoull(field.second));
            }
          }

          if (!(s && t)) {
            response->write(SimpleWeb::StatusCode::client_error_bad_request,
                "Request needs to contain the parameters: s, t");
          }
          std::stringstream result;

          std::optional<Route> route = {};
          std::optional<Route> route2 = {};
          Config conf1{ LengthConfig{ 0 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } };
          Config conf2{ LengthConfig{ 0 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } };
          double shared = 1.0;
          double threshold = 0.3;

          size_t counter = 0;
          while (shared > threshold) {
            if (++counter % 100 == 0) {
              threshold += 0.1;
            }
            conf1 = generateRandomConfig();
            route = dijkstra.findBestRoute(NodePos{ *s }, NodePos{ *t }, conf1);
            if (!route) {
              response->write(SimpleWeb::StatusCode::client_error_not_found, "Did not find route");
              return;
            }

            conf2 = generateRandomConfig();
            route2 = dijkstra.findBestRoute(NodePos{ *s }, NodePos{ *t }, conf2);
            shared = calculateSharing(*route, *route2);
          }

          result << R"({ "config1": ")" << std::round(conf1.length * 100) << "/"
                 << std::round(conf1.height * 100) << "/" << std::round(conf1.unsuitability * 100)
                 << R"(", )";
          result << R"( "route1":  )" << routeToJson(*route, g) << ", ";

          result << R"( "config2": ")" << std::round(conf2.length * 100) << "/"
                 << std::round(conf2.height * 100) << "/" << std::round(conf2.unsuitability * 100)
                 << R"(", )";
          result << R"( "route2":  )" << routeToJson(*route2, g) << ", ";
          result << R"( "shared":  )" << std::round(shared * 100) << "}";

          SimpleWeb::CaseInsensitiveMultimap header;
          header.emplace("Content-Type", "application/json");
          response->write(SimpleWeb::StatusCode::success_ok, result, header);

        };

  std::cout << "Starting web server at http://localhost:" << server.config.port << '\n';
  server.start();
}

int testGraph(Graph& g)
{
  Dijkstra d = g.createDijkstra();
  NormalDijkstra n = g.createNormalDijkstra(true);
  std::random_device rd{};
  std::uniform_int_distribution<size_t> dist(0, g.getNodeCount() - 1);
  Config c{ LengthConfig{ 0.33 }, HeightConfig{ 0.33 }, UnsuitabilityConfig{ 0.33 } };

  size_t route = 0;
  size_t noRoute = 0;

  for (int i = 0; i < 100; ++i) {
    NodePos from{ dist(rd) };
    NodePos to{ dist(rd) };

    auto dRoute = d.findBestRoute(from, to, c);
    auto nRoute = n.findBestRoute(from, to, c);

    if (dRoute && nRoute) {
      ++route;
      if (std::abs(dRoute->costs * c - nRoute->costs * c) > 0.01) {
        std::cout << "cost differ in route from " << from << " to " << to << '\n';
        std::cout << "dLength: " << dRoute->costs.length << ", nLength: " << nRoute->costs.length
                  << '\n';
        std::cout << "dHeight: " << dRoute->costs.height << ", nHeight: " << nRoute->costs.height
                  << '\n';
        std::cout << "dUnsuitability: " << dRoute->costs.unsuitability
                  << ", nUnsuitability: " << nRoute->costs.unsuitability << '\n';
        std::cout << "total cost d: " << dRoute->costs * c
                  << ", total cost n: " << nRoute->costs * c << '\n';

        std::unordered_set<NodeId> nodeIds;
        std::transform(nRoute->edges.begin(), nRoute->edges.end(),
            std::inserter(nodeIds, begin(nodeIds)), [](const auto& e) {
              const auto& edge = Edge::getEdge(e);
              return edge.getSourceId();
            });

        auto idToPos = g.getNodePosByIds(nodeIds);
        std::vector<const Node*> nodeLevels;
        std::transform(nRoute->edges.begin(), nRoute->edges.end(), std::back_inserter(nodeLevels),
            [&idToPos](const auto& e) {
              const auto edge = Edge::getEdge(e);
              return idToPos[edge.getSourceId()];
            });

        for (size_t i = 2; i < nodeLevels.size(); ++i) {
          auto nextToLast = nodeLevels[i - 2];
          auto last = nodeLevels[i - 1];
          auto current = nodeLevels[i];

          if (nextToLast->getLevel() > last->getLevel() && last->getLevel() < current->getLevel()) {
            auto nextToLastPos = g.getNodePos(nextToLast);
            auto lastPos = g.getNodePos(last);
            auto currentPos = g.getNodePos(current);
            const auto dTest = d.findBestRoute(nextToLastPos, currentPos, c);
            const auto nTest = n.findBestRoute(nextToLastPos, currentPos, c);
            if (!(dTest->costs * c <= nTest->costs * c)) {
              std::cout << "did not find correct subpath between " << nextToLastPos << " and "
                        << currentPos << " at index " << i << '\n';

              std::cout << '\n'
                        << "Normal dijkstra needs length " << nTest->costs.length << " height "
                        << nTest->costs.height << " road " << nTest->costs.unsuitability << '\n';

              std::cout << "CH dijkstra needs     length " << dTest->costs.length << " height "
                        << dTest->costs.height << " road " << dTest->costs.unsuitability << '\n';

              std::cout << nextToLastPos << " has level " << nextToLast->getLevel() << '\n';
              std::cout << lastPos << " has level " << last->getLevel() << '\n';
              std::cout << currentPos << " has level " << current->getLevel() << '\n';

              auto outEdges = g.getOutgoingEdgesOf(nextToLastPos);
              std::cout << "Outgoing edges of " << nextToLastPos << '\n';
              for (const auto& edge : outEdges) {
                std::cout << "Id " << edge.id << " Target " << edge.end << " length "
                          << edge.cost.length << ", height " << edge.cost.height
                          << ", road: " << edge.cost.unsuitability << '\n';
              }
              outEdges = g.getOutgoingEdgesOf(g.getNodePos(last));
              std::cout << "Outgoing edges of " << g.getNodePos(last) << '\n';
              for (const auto& edge : outEdges) {
                std::cout << "Id " << edge.id << " Target " << edge.end << " length "
                          << edge.cost.length << ", height " << edge.cost.height
                          << ", road: " << edge.cost.unsuitability << '\n';
              }

              std::cout << "------------------ ROUTES --------------------" << '\n';
              std::cout << "Normal Route " << '\n';
              for (const auto& e : nTest->edges) {
                const auto edge = Edge::getEdge(e);
                std::cout << "length " << edge.getCost().length << " height "
                          << edge.getCost().height << " road " << edge.getCost().unsuitability
                          << '\n';
              }

              std::cout << "CH Route" << '\n';
              for (const auto& edge : dTest->edges) {
                std::cout << "length " << edge.getCost().length << " height "
                          << edge.getCost().height << " road " << edge.getCost().unsuitability
                          << '\n';
              }

              return -1;
            }
          }
        }

        return 1;
      }
    } else if (nRoute && !dRoute) {
      std::cout << "Only Normal dijkstra found route form " << from << " to " << to << "!" << '\n';
      return 1;
    } else {
      ++noRoute;
    }
  }
  std::cout << "Compared " << route << " routes" << '\n';
  std::cout << "Did not find a route in " << noRoute << " cases" << '\n';
  return 0;
}

namespace po = boost::program_options;
int main(int argc, char* argv[])
{
  std::cout.imbue(std::locale(""));

  std::string textFileName{};
  std::string binFileName{};
  std::string saveFileName{};
  unsigned short contractionPercent;

  po::options_description loading{ "loading options" };
  loading.add_options()(
      "text,t", po::value<std::string>(&textFileName), "load graph from text file")(
      "bin,b", po::value<std::string>(&binFileName), "load graph form binary file");

  po::options_description action{ "actions" };
  action.add_options()("contract,c", "contract graph");
  action.add_options()("percent,p",
      po::value<unsigned short>(&contractionPercent)->default_value(98),
      "How far the graph should be contracted");
  action.add_options()("stats", "print statistics while contracting");
  action.add_options()("dijkstra,d", "start interactive dijkstra in cli");
  action.add_options()("web,w", "start webserver for interaction via browser");
  action.add_options()("save", po::value<std::string>(&saveFileName), "save graph to binary file");
  action.add_options()("test", "runs normal dijkstra and CH dijktra for comparison");

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
    bool printStats = vm.count("stats") > 0;
    std::cout << "Start contracting" << '\n';
    g = contractGraph(g, 100 - contractionPercent, printStats);
  }

  if (!saveFileName.empty()) {
    std::cout << "Saving" << '\n';
    saveToBinaryFile(g, saveFileName);
  }

  if (vm.count("dijkstra") > 0) {
    cliDijkstra(g);
  }

  if (vm.count("web") > 0) {
    runWebServer(g);
  }

  if (vm.count("test") > 0) {
    return testGraph(g);
  }

  return 0;
}
