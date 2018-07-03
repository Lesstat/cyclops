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
#include "graph_loading.hpp"
#include "grid.hpp"
#include "loginfo.hpp"
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

Graph contractGraph(Graph& g, double rest, bool printStats)
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

    auto dijkstra = g.createDijkstra();
    auto bestLengthRoute = dijkstra.findBestRoute(NodePos{ *s }, NodePos{ *t },
        Config{ LengthConfig{ 1 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } });
    if (!bestLengthRoute) {
      return;
    }
    auto bestHeightRoute = dijkstra.findBestRoute(NodePos{ *s }, NodePos{ *t },
        Config{ LengthConfig{ 0 }, HeightConfig{ 1 }, UnsuitabilityConfig{ 0 } });
    auto bestUnsuitabilityRoute = dijkstra.findBestRoute(NodePos{ *s }, NodePos{ *t },
        Config{ LengthConfig{ 0 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 1 } });

    auto bestLength = bestLengthRoute->costs.values[0];
    auto bestHeight = bestHeightRoute->costs.values[1];
    auto bestUnsuitability = bestUnsuitabilityRoute->costs.values[2];

    auto maxConfigParam = std::max({ bestLength, bestHeight, bestUnsuitability });

    auto lengthFactor = maxConfigParam / bestLength;
    auto heightFactor = maxConfigParam / bestHeight;
    auto unsuitFactor = maxConfigParam / bestUnsuitability;

    Config c{ LengthConfig{ (static_cast<double>(*length) / 100.0) },
      HeightConfig{ (static_cast<double>(*height) / 100.0) },
      UnsuitabilityConfig{ (static_cast<double>(*unsuitability) / 100.0) } };

    c.values[0] *= lengthFactor;
    c.values[2] *= heightFactor;
    c.values[2] *= unsuitFactor;

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

  for (int i = 0; i < 100; ++i) {
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
      ++route;
      if (std::abs(dRoute->costs * c - nRoute->costs * c) > 0.1) {
        std::cout << '\n' << "cost differ in route from " << from << " to " << to << '\n';
        std::cout << "dLength: " << dRoute->costs.values[0]
                  << ", nLength: " << nRoute->costs.values[0] << '\n';
        std::cout << "dHeight: " << dRoute->costs.values[1]
                  << ", nHeight: " << nRoute->costs.values[1] << '\n';
        std::cout << "dUnsuitability: " << dRoute->costs.values[2]
                  << ", nUnsuitability: " << nRoute->costs.values[2] << '\n';
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
                        << "Normal dijkstra needs length " << nTest->costs.values[0] << " height "
                        << nTest->costs.values[1] << " road " << nTest->costs.values[2] << '\n';

              std::cout << "CH dijkstra needs     length " << dTest->costs.values[0] << " height "
                        << dTest->costs.values[1] << " road " << dTest->costs.values[2] << '\n';

              std::cout << nextToLastPos << " has level " << nextToLast->getLevel() << '\n';
              std::cout << lastPos << " has level " << last->getLevel() << '\n';
              std::cout << currentPos << " has level " << current->getLevel() << '\n';

              auto outEdges = g.getOutgoingEdgesOf(nextToLastPos);
              std::cout << "Outgoing edges of " << nextToLastPos << '\n';
              for (const auto& edge : outEdges) {
                std::cout << "Id " << edge.id << " Target " << edge.end << " length "
                          << edge.cost.values[0] << ", height " << edge.cost.values[1]
                          << ", road: " << edge.cost.values[2] << '\n';
              }
              outEdges = g.getOutgoingEdgesOf(g.getNodePos(last));
              std::cout << "Outgoing edges of " << g.getNodePos(last) << '\n';
              for (const auto& edge : outEdges) {
                std::cout << "Id " << edge.id << " Target " << edge.end << " length "
                          << edge.cost.values[0] << ", height " << edge.cost.values[1]
                          << ", road: " << edge.cost.values[2] << '\n';
              }

              std::cout << "------------------ ROUTES --------------------" << '\n';
              std::cout << "Normal Route " << '\n';
              for (const auto& e : nTest->edges) {
                const auto edge = Edge::getEdge(e);
                std::cout << "length " << edge.getCost().values[0] << " height "
                          << edge.getCost().values[1] << " road " << edge.getCost().values[2]
                          << '\n';
              }

              std::cout << "CH Route" << '\n';
              for (const auto& edge : dTest->edges) {
                std::cout << "length " << edge.getCost().values[0] << " height "
                          << edge.getCost().values[1] << " road " << edge.getCost().values[2]
                          << '\n';
              }

              return -1;
            }
          }
        }

        return 1;
      }
      std::cout << '+';
      std::cout.flush();
    } else if (nRoute && !dRoute) {
      std::cout << "Only Normal dijkstra found route form " << from << " to " << to << "!" << '\n';
      return 1;
    } else {
      ++noRoute;
    }
  }
  std::cout << '\n';
  std::cout << "Compared " << route << " routes" << '\n';
  std::cout << "Did not find a route in " << noRoute << " cases" << '\n';
  return 0;
}

namespace po = boost::program_options;
int main(int argc, char* argv[])
{
  std::cout.imbue(std::locale(""));

  std::string loadFileName{};
  std::string saveFileName{};
  double contractionPercent;

  po::options_description loading{ "loading options" };
  loading.add_options()(
      "text,t", po::value<std::string>(&loadFileName), "load graph from text file")(
      "bin,b", po::value<std::string>(&loadFileName), "load graph form binary file")(
      "multi,m", po::value<std::string>(&loadFileName), "load graph from multiple files");

  po::options_description action{ "actions" };
  action.add_options()("contract,c", "contract graph");
  action.add_options()("percent,p", po::value<double>(&contractionPercent)->default_value(98),
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
