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

#include "dijkstra.hpp"
#include "enumerate_optimals.hpp"
#include "graph_loading.hpp"
#include "grid.hpp"
#include "loginfo.hpp"
#include "ndijkstra.hpp"
#include "routeComparator.hpp"
#include "scaling_triangulation.hpp"
#include "server_http.hpp"
#include "webUtilities.hpp"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <random>

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

      std::cout << "Route is " << route.costs.values[0] << "m long" << '\n';
      std::cout << "Route has " << route.costs.values[1] << "m absolute height difference" << '\n';
      std::cout << "Route has " << route.costs.values[2] << " unsuitability costs" << '\n';

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

  HttpServer server;
  server.config.port = 8080;
  server.config.thread_pool_size
      = std::thread::hardware_concurrency() > 0 ? std::thread::hardware_concurrency() : 1;

  server.default_resource["GET"] = [](Response response, Request /*request*/) {
    response->write(SimpleWeb::StatusCode::client_error_not_found, "No matching handler found");
  };

  server.resource["^/web/?.*"]["GET"] = [](Response response, Request request) {
    Logger::initLogger();
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
    Logger::initLogger();
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
      return;
    }
    auto pos = grid.findNextNode(Lat{ lat }, Lng{ lng });
    SimpleWeb::CaseInsensitiveMultimap header;
    header.emplace("Content-Type", "text/plain");
    response->write(SimpleWeb::StatusCode::success_ok, std::to_string(pos->get()), header);
  };

  server.resource["^/route"]["GET"] = [&g](Response response, Request request) {
    auto log = Logger::initLogger();

    std::optional<size_t> s{}, t{}, length{}, height{}, unsuitability{};
    extractQueryFields(request->parse_query_string(), s, t, length, height, unsuitability);
    if (s > g.getNodeCount() || t > g.getNodeCount()) {
      response->write(
          SimpleWeb::StatusCode::client_error_bad_request, "Request contains illegal node ids");
      return;
    }

    if (!(s && t && length && height && unsuitability)) {
      response->write(SimpleWeb::StatusCode::client_error_bad_request,
          "Request needs to contain the parameters: s, t, length, height, unsuitability");
      return;
    }

    try {
      auto dijkstra = g.createDijkstra();

      Dijkstra::ScalingFactor f;
      dijkstra.calcScalingFactor(NodePos{ *s }, NodePos{ *t }, f);

      Config c{ LengthConfig{ (static_cast<double>(*length) / 100.0) },
        HeightConfig{ (static_cast<double>(*height) / 100.0) },
        UnsuitabilityConfig{ (static_cast<double>(*unsuitability) / 100.0) } };

      for (size_t i = 0; i < DIMENSION; ++i) {
        c.values[i] *= f[i];
      }

      auto start = std::chrono::high_resolution_clock::now();
      auto route = dijkstra.findBestRoute(NodePos{ *s }, NodePos{ *t }, c);
      auto end = std::chrono::high_resolution_clock::now();
      size_t dur = std::chrono::duration_cast<ms>(end - start).count();
      *log << "Dijkstra took " << dur << "ms"
           << "\\n";

      std::cout << "Finding the route took " << dur << "ms" << '\n';
      if (route) {

        auto json = routeToJson(*route, g, true);
        SimpleWeb::CaseInsensitiveMultimap header;
        header.emplace("Content-Type", "application/json");
        response->write(SimpleWeb::StatusCode::success_ok, json, header);
      }
      response->write(SimpleWeb::StatusCode::client_error_not_found, "Did not find route");
    } catch (std::exception& e) {
      response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
    }
  };

  server.resource["^/scaled"]["GET"] = [&g](Response response, Request request) {
    auto log = Logger::initLogger();

    std::optional<size_t> s{}, t{}, maxSplits{}, dummy{}, maxLevel{}, maxOverlap{};
    bool splitByLevel{ false };

    auto queryParams = request->parse_query_string();
    extractQueryFields(queryParams, s, t, dummy, dummy, dummy);
    if (s > g.getNodeCount() || t > g.getNodeCount()) {
      response->write(
          SimpleWeb::StatusCode::client_error_bad_request, "Request contains illegal node ids");
      return;
    }
    if (!(s && t)) {
      response->write(SimpleWeb::StatusCode::client_error_bad_request,
          "Request needs to contain the parameters: s, t");
      return;
    }
    for (const auto& param : queryParams) {
      if (param.first == "maxSplits") {
        maxSplits = stoull(param.second);
      } else if (param.first == "maxLevel") {
        maxLevel = stoull(param.second);
      } else if (param.first == "splitByLevel" && param.second == "true") {
        splitByLevel = true;
      } else if (param.first == "maxOverlap") {
        maxOverlap = stoull(param.second);
      }
    }

    *log << "from " << *s << " to " << *t << "\\n";

    auto d = g.createDijkstra();
    auto triangulation = scaledTriangulation(d, NodePos{ *s }, NodePos{ *t },
        maxSplits.value_or(10), maxLevel, splitByLevel, maxOverlap.value_or(95) / 100.0);
    auto& points = triangulation.points;
    auto& triangles = triangulation.triangles;
    std::stringstream result;

    auto selCount
        = std::count_if(points.begin(), points.end(), [](const auto& p) { return p.selected; });
    std::cout << "Selected " << selCount << " routes!" << '\n';

    result << "{ \"points\": [";
    bool first = true;
    for (auto& p : points) {
      if (first) {
        first = false;
      } else {
        result << ",";
      }
      result << "{ \"conf\": \"" << p.p << "\",";
      result << "\"route\":" << routeToJson(p.r, g) << ",";
      result << "\"selected\": " << p.selected;
      result << "}";
    }
    result << "],";
    result << "\"triangles\": [";
    first = true;
    for (auto& t : triangles) {
      if (first) {
        first = false;
      } else {
        result << ",";
      }
      result << "{ \"point1\":" << t.point1 << ",";
      result << "\"point2\":" << t.point2 << ",";
      result << "\"point3\":" << t.point3 << ",";
      result << "\"noChildren\":" << t.noChildren << ",";
      result << "\"noMoreRoutes\":" << t.noMoreRoutes;
      result << "}";
    }

    result << "],";
    result << "\"debug\":\"" << log->getInfo() << "\" ";

    result << "}";

    SimpleWeb::CaseInsensitiveMultimap header;
    header.emplace("Content-Type", "application/json");
    response->write(SimpleWeb::StatusCode::success_ok, result, header);
  };

  server.resource["^/enumerate"]["GET"] = [&g](Response response, Request request) {
    std::cout << "enumerate request"
              << "\n";
    auto log = Logger::initLogger();

    std::optional<size_t> s{}, t{}, dummy{}, maxOverlap{}, maxRoutes{};

    auto queryParams = request->parse_query_string();
    extractQueryFields(queryParams, s, t, dummy, dummy, dummy);
    for (const auto& param : queryParams) {
      if (param.first == "maxRoutes") {
        maxRoutes = stoull(param.second);
      } else if (param.first == "maxOverlap") {
        maxOverlap = stoull(param.second);
      }
    }
    if (s > g.getNodeCount() || t > g.getNodeCount()) {
      response->write(
          SimpleWeb::StatusCode::client_error_bad_request, "Request contains illegal node ids");
      return;
    }
    if (!(s && t && maxOverlap && maxRoutes)) {
      response->write(SimpleWeb::StatusCode::client_error_bad_request,
          "Request needs to contain the parameters: s, t, maxOverlap, maxRoutes");
      return;
    }
    *log << "from " << *s << " to " << *t << "\\n";
    try {

      std::cout << "starting computation"
                << "\n";
      auto [routes, configs] = EnumerateOptimals(g, *maxOverlap / 100.0, *maxRoutes)
                                   .find(NodePos{ *s }, NodePos{ *t });

      std::stringstream result;

      std::cout << "building json for " << routes.size() << " routes" << '\n';

      result << "{ \"points\": [";
      for (size_t i = 0; i < routes.size(); ++i) {
        if (i > 0) {
          result << ",";
        }
        result << "{ \"conf\": \"" << configs[i] << "\",";
        result << "\"route\":" << routeToJson(routes[i], g) << ",";
        result << "\"selected\": "
               << "true";
        result << "}";
      }
      result << "],";
      result << "\"debug\":\"" << log->getInfo() << "\" ";

      result << "}";

      SimpleWeb::CaseInsensitiveMultimap header;
      header.emplace("Content-Type", "application/json");
      response->write(SimpleWeb::StatusCode::success_ok, result, header);
    } catch (std::exception& e) {
      response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
    }
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
  Config c{ LengthConfig{ 1.0 / 3.0 }, HeightConfig{ 1.0 / 3.0 },
    UnsuitabilityConfig{ 1.0 / 3.0 } };

  size_t route = 0;
  size_t noRoute = 0;

  size_t dTime = 0;
  size_t nTime = 0;

  for (int i = 0; i < 1000; ++i) {
    NodePos from{ dist(rd) };
    NodePos to{ dist(rd) };

    auto dStart = std::chrono::high_resolution_clock::now();
    auto dRoute = d.findBestRoute(from, to, c);
    auto dEnd = std::chrono::high_resolution_clock::now();
    auto nRoute = n.findBestRoute(from, to, c);
    auto nEnd = std::chrono::high_resolution_clock::now();

    if (dRoute && nRoute) {
      auto normalTime = std::chrono::duration_cast<ms>(nEnd - dEnd).count();
      auto chTime = std::chrono::duration_cast<ms>(dEnd - dStart).count();
      std::cout << "ND/CH: " << normalTime << "/" << chTime << " = "
                << (chTime > 0 ? normalTime / chTime : 999999999999999) << '\n';
      dTime += chTime;
      nTime += normalTime;
      ++route;
      if (std::abs(dRoute->costs * c - nRoute->costs * c) > 0.1) {
        std::cout << '\n'
                  << "cost differ in route from " << from << " (" << g.getNode(from).id() << ") to "
                  << to << " (" << g.getNode(to).id() << ")" << '\n';
        std::cout << "Edge count: " << nRoute->edges.size() << '\n';
        std::cout << "dLength: " << dRoute->costs.values[0]
                  << ", nLength: " << nRoute->costs.values[0] << '\n';
        std::cout << "dHeight: " << dRoute->costs.values[1]
                  << ", nHeight: " << nRoute->costs.values[1] << '\n';
        std::cout << "dUnsuitability: " << dRoute->costs.values[2]
                  << ", nUnsuitability: " << nRoute->costs.values[2] << '\n';
        std::cout << "total cost d: " << dRoute->costs * c
                  << ", total cost n: " << nRoute->costs * c << '\n';
        std::ofstream wholeRoute{ "/tmp/whole-" + std::to_string(from) + "-" + std::to_string(to)
          + ".dot" };
        printRoutes(wholeRoute, g, *nRoute, *dRoute, c);
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

        size_t pos = 1;
        while (pos < nodeLevels.size()) {
          std::cout << "Trying pos " << pos << "\n";
          const Node* start = nullptr;
          start = nodeLevels[pos];

          if (start) {
            auto nextToLastPos = g.getNodePos(start);
            auto currentPos = to;
            const auto dTest = d.findBestRoute(nextToLastPos, currentPos, c);
            const auto nTest = n.findBestRoute(nextToLastPos, currentPos, c);
            if (!(dTest->costs * c <= nTest->costs * c)) {
              std::cout << "start id: " << start->id() << "\n";
              std::cout << "did not find correct subpath between " << nextToLastPos << " and "
                        << currentPos << " at index " << i << '\n';

              std::cout << '\n'
                        << "Normal dijkstra needs length " << nTest->costs.values[0] << " height "
                        << nTest->costs.values[1] << " road " << nTest->costs.values[2] << '\n';

              std::cout << "CH dijkstra needs     length " << dTest->costs.values[0] << " height "
                        << dTest->costs.values[1] << " road " << dTest->costs.values[2] << '\n';

              std::ofstream falsePart{ "/tmp/false-" + std::to_string(from) + "-"
                + std::to_string(to) + ".dot" };
              printRoutes(falsePart, g, *nTest, *dTest, c);

              pos++;
            } else {
              std::cout << "equal routes"
                        << "\n";
              return 1;
            }
          }
        }
      }
    } else if (nRoute && !dRoute) {
      std::cout << "Only Normal dijkstra found route form " << from << " to " << to << "!" << '\n';
      return 1;
    } else {
      ++noRoute;
    }
    std::cout << '+';
    std::cout.flush();
  }

  std::cout << '\n';
  std::cout << "Compared " << route << " routes" << '\n';
  std::cout << "Did not find a route in " << noRoute << " cases" << '\n';
  std::cout << "average speed up is " << static_cast<double>(nTime) / dTime << '\n';
  return 0;
}

namespace po = boost::program_options;
int main(int argc, char* argv[])
{
  std::cout.imbue(std::locale(""));

  std::string loadFileName{};
  std::string saveFileName{};

  po::options_description loading{ "loading options" };
  loading.add_options()(
      "text,t", po::value<std::string>(&loadFileName), "load graph from text file")(
      "bin,b", po::value<std::string>(&loadFileName), "load graph form binary file")(
      "multi,m", po::value<std::string>(&loadFileName), "load graph from multiple files");

  po::options_description action{ "actions" };
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
  if (vm.count("text") > 0) {
    g = loadGraphFromTextFile(loadFileName);
  } else if (vm.count("bin") > 0) {
    g = loadGraphFromBinaryFile(loadFileName);
  } else if (vm.count("multi") > 0) {
    g = readMultiFileGraph(loadFileName);
  } else {
    std::cout << "No input file given" << '\n';
    std::cout << all << '\n';
    return 0;
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
