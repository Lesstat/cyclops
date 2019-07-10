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
#include "url_parsing.hpp"
#include "webUtilities.hpp"

#include "server_http.hpp"
#include "json/json.h"

#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <random>

template <int Dim> void saveToBinaryFile(Graph<Dim>& ch, std::string& filename)
{
  {
    std::ofstream ofs(filename, std::ios::binary);
    boost::archive::binary_oarchive oa { ofs };
    oa << ch;
  }
}

template <int Dim> void runWebServer(Graph<Dim>& g, unsigned short port, size_t max_refinements)
{
  using HttpServer = SimpleWeb::Server<SimpleWeb::HTTP>;
  using Response = std::shared_ptr<HttpServer::Response>;
  using Request = std::shared_ptr<HttpServer::Request>;

  using Dijkstra = Dijkstra<Dim>;
  using Config = Config<Dim>;

  Grid grid = g.createGrid();

  HttpServer server;
  server.config.port = port;
  server.config.thread_pool_size
      = std::thread::hardware_concurrency() > 0 ? std::thread::hardware_concurrency() : 1;

  server.default_resource["GET"] = [](Response response, Request /*request*/) {
    SimpleWeb::CaseInsensitiveMultimap header;
    header.emplace("Location", "/web");
    response->write(
        SimpleWeb::StatusCode::redirection_temporary_redirect, "No matching handler found", header);
  };

  server.resource["^/web/?.*"]["GET"] = [](Response response, Request request) {
    Logger::initLogger();
    auto web_root_path = boost::filesystem::canonical("web");

    std::string pathWithoutWeb {};
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

    std::ifstream ifs {};
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
    auto pos = grid.findNextNode(Lat { lat }, Lng { lng });
    SimpleWeb::CaseInsensitiveMultimap header;
    header.emplace("Content-Type", "text/plain");
    response->write(SimpleWeb::StatusCode::success_ok, std::to_string(pos->get()), header);
  };

  server.resource["^/graph_coords"]["GET"] = [&grid](Response response, Request /*request*/) {
    auto b_box = grid.bounding_box();

    Json::Value result;
    result["lat_min"] = b_box.lat_min.get();
    result["lat_max"] = b_box.lat_max.get();
    result["lng_min"] = b_box.lng_min.get();
    result["lng_max"] = b_box.lng_max.get();

    SimpleWeb::CaseInsensitiveMultimap header;
    header.emplace("Content-Type", "application/json");

    Json::StreamWriterBuilder builder;
    response->write(SimpleWeb::StatusCode::success_ok, Json::writeString(builder, result), header);
  };

  server.resource["^/route"]["GET"] = [&g](Response response, Request request) {
    auto log = Logger::initLogger();

    std::optional<uint32_t> s {}, t {}, length {}, height {}, unsuitability {};
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

      typename Dijkstra::ScalingFactor f;
      dijkstra.calcScalingFactor(NodePos { *s }, NodePos { *t }, f);

      Config c { LengthConfig { (static_cast<double>(*length) / 100.0) },
        HeightConfig { (static_cast<double>(*height) / 100.0) },
        UnsuitabilityConfig { (static_cast<double>(*unsuitability) / 100.0) } };

      for (size_t i = 0; i < Dim; ++i) {
        c.values[i] *= f[i];
      }

      auto start = std::chrono::high_resolution_clock::now();
      auto route = dijkstra.findBestRoute(NodePos { *s }, NodePos { *t }, c);
      auto end = std::chrono::high_resolution_clock::now();
      size_t dur = std::chrono::duration_cast<ms>(end - start).count();
      *log << "Dijkstra took " << dur << "ms"
           << "\n \n";

      std::cout << "Finding the route took " << dur << "ms" << '\n';
      if (route) {

        auto json = routeToJson(*route, g, true);
        SimpleWeb::CaseInsensitiveMultimap header;
        header.emplace("Content-Type", "application/json");

        Json::StreamWriterBuilder builder;
        response->write(
            SimpleWeb::StatusCode::success_ok, Json::writeString(builder, json), header);
      }
      response->write(SimpleWeb::StatusCode::client_error_not_found, "Did not find route");
    } catch (std::exception& e) {
      response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
    }
  };

  server.resource["^/enumerate"]["GET"]
      = [&g, max_refinements](Response response, Request request) {
          auto log = Logger::initLogger();

          std::optional<uint32_t> s {}, t {}, dummy {}, maxOverlap {}, maxRoutes {};
          std::vector<ImportantMetric> important_metrics;

          auto queryParams = request->parse_query_string();
          extractQueryFields(queryParams, s, t, dummy, dummy, dummy);
          for (const auto& param : queryParams) {
            if (param.first == "maxRoutes") {
              maxRoutes = stoull(param.second);
            } else if (param.first == "maxOverlap") {
              maxOverlap = stoull(param.second);
            } else if (param.first == "important") {
              try {
                important_metrics = parse_important_metric_list(param.second);
              } catch (std::exception& e) {
                response->write(SimpleWeb::StatusCode::client_error_bad_request, e.what());
                return;
              }
            }
          }
          if (s > g.getNodeCount() || t > g.getNodeCount()) {
            response->write(SimpleWeb::StatusCode::client_error_bad_request,
                "Request contains illegal node ids");
            return;
          }
          if (!(s && t && maxOverlap && maxRoutes)) {
            response->write(SimpleWeb::StatusCode::client_error_bad_request,
                "Request needs to contain the parameters: s, t, maxOverlap, maxRoutes");
            return;
          }
          uint32_t log_s = *s;
          uint32_t log_t = *t;
          *log << "enumerating routes from " << log_s << " to " << log_t << "\n";

          if (*maxRoutes > max_refinements) {
            size_t log_max_routes = *maxRoutes;
            *log << "reduced max refinements from " << log_max_routes << " to " << max_refinements
                 << " because of server options."
                 << "\n";
            maxRoutes = max_refinements;
          }

          auto overlap = *maxOverlap / 100.0;
          auto [routes, configs] = [&]() {
            if (important_metrics.empty()) {

              EnumerateOptimals<Dim, SimilarityPrio> enumerate(&g, *maxRoutes);
              enumerate.set_overlap(overlap);

              enumerate.find(NodePos { *s }, NodePos { *t });
              return enumerate.recommend_routes(false);
            } else {

              auto slacks = important_metrics_to_array<Dim>(important_metrics);

              EnumerateOptimals<Dim, SimilarityPrioExcludeIrrelevant> enumerate(&g, *maxRoutes);
              enumerate.set_overlap(overlap);
              enumerate.set_slack(slacks);

              enumerate.find(NodePos { *s }, NodePos { *t });
              return enumerate.recommend_routes(false);
            }
          }();

          try {

            Json::Value result;
            Json::Value points(Json::arrayValue);

            for (size_t i = 0; i < routes.size(); ++i) {

              Json::Value route;
              Json::Value conf(Json::arrayValue);

              for (const auto& v : configs[i].values) {
                conf.append(v);
              }
              route["conf"] = conf;

              route["route"] = routeToJson(routes[i], g);
              route["selected"] = true;

              points.append(route);
            }

            result["points"] = points;
            result["debug"] = log->getInfo();

            SimpleWeb::CaseInsensitiveMultimap header;
            header.emplace("Content-Type", "application/json");

            Json::StreamWriterBuilder builder;

            response->write(
                SimpleWeb::StatusCode::success_ok, Json::writeString(builder, result), header);
          } catch (std::exception& e) {
            response->write(SimpleWeb::StatusCode::server_error_internal_server_error, e.what());
          }
        };

  std::cout << "Starting web server at http://localhost:" << server.config.port << '\n';
  server.start();
}

template <int Dim> int testGraph(Graph<Dim>& g)
{
  using Config = Config<Dim>;
  using Edge = Edge<Dim>;

  auto d = g.createDijkstra();
  auto n = g.createNormalDijkstra(true);
  std::random_device rd {};
  std::uniform_int_distribution<size_t> dist(0, g.getNodeCount() - 1);
  Config c { std::vector<double>(Dim, 1.0 / Dim) };

  size_t route = 0;
  size_t noRoute = 0;

  size_t dTime = 0;
  size_t nTime = 0;

  size_t dPops = 0;
  size_t nPops = 0;

  for (int i = 0; i < 1000; ++i) {
    NodePos from { dist(rd) };
    NodePos to { dist(rd) };

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
      dPops += d.pqPops;
      nPops += n.pqPops;
      ++route;
      if (std::abs(dRoute->costs * c - nRoute->costs * c) > 0.1) {
        std::cout << '\n'
                  << "cost differ in route from " << from << " (" << g.getNode(from).id() << ") to "
                  << to << " (" << g.getNode(to).id() << ")" << '\n';
        std::cout << "Edge count: " << nRoute->edges.size() << '\n';
        for (size_t i = 0; i < Dim; ++i) {
          std::cout << "cost " << i << " d: " << dRoute->costs.values[i] << ", cost " << i
                    << " n:" << nRoute->costs.values[i] << '\n';
        }

        std::cout << "total cost d: " << dRoute->costs * c
                  << ", total cost n: " << nRoute->costs * c << '\n';
        std::ofstream wholeRoute { "/tmp/whole-" + std::to_string(from) + "-" + std::to_string(to)
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
          std::cout << "Trying pos " << pos << '\n';
          const Node* start = nullptr;
          start = nodeLevels[pos];

          if (start) {
            auto nextToLastPos = g.getNodePos(start);
            auto currentPos = to;
            const auto dTest = d.findBestRoute(nextToLastPos, currentPos, c);
            const auto nTest = n.findBestRoute(nextToLastPos, currentPos, c);
            if (!(dTest->costs * c <= nTest->costs * c)) {
              std::cout << "start id: " << start->id() << '\n';
              std::cout << "did not find correct subpath between " << nextToLastPos << " and "
                        << currentPos << " at index " << i << '\n';

              std::cout << '\n' << "Normal dijkstra needs: ";
              for (size_t i = 0; i < Dim; ++i) {
                std::cout << nTest->costs.values[i] << ", ";
              }

              std::cout << '\n' << "CH dijkstra needs: ";
              for (size_t i = 0; i < Dim; ++i) {
                std::cout << nTest->costs.values[i] << ", ";
              }
              std::cout << '\n';

              std::ofstream falsePart { "/tmp/false-" + std::to_string(from) + "-"
                + std::to_string(to) + ".dot" };
              printRoutes(falsePart, g, *nTest, *dTest, c);

              pos++;
            } else {
              std::cout << "equal routes" << '\n';
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
  std::cout << "Average CH-Dijkstra time: " << static_cast<double>(dTime) / route << "ms" << '\n';
  std::cout << "Average    Dijkstra time: " << static_cast<double>(nTime) / route << "ms" << '\n';
  std::cout << "Average CH-Dijkstra pops: " << static_cast<double>(dPops) / route << '\n';
  std::cout << "Average    Dijkstra pops: " << static_cast<double>(nPops) / route << '\n';
  std::cout << "PQ-Pops ratio: " << static_cast<double>(nPops) / dPops << '\n';
  return 0;
}

namespace po = boost::program_options;

template <int Dim>
int run(po::variables_map& vm, std::string& loadFileName, std::string& saveFileName,
    unsigned short port, size_t max_refinements)
{

  Graph<Dim> g { std::vector<Node>(), std::vector<Edge<Dim>>() };
  if (vm.count("text") > 0) {
    bool zipped_input = vm.count("zi") > 0;
    g = loadGraphFromTextFile<Dim>(loadFileName, zipped_input);
  } else if (vm.count("bin") > 0) {
    g = loadGraphFromBinaryFile<Dim>(loadFileName);
  } else {
    std::cout << "No input file given" << '\n';
    std::cout << "Maybe try --help" << '\n';
    return 0;
  }
  if (!saveFileName.empty()) {
    std::cout << "Saving" << '\n';
    saveToBinaryFile(g, saveFileName);
  }

  if (vm.count("test") > 0) {
    return testGraph(g);
  }

  if (vm.count("web") > 0) {
    runWebServer(g, port, max_refinements);
  }
  return 0;
}

int main(int argc, char* argv[])
{
  std::cout.imbue(std::locale(""));

  std::string loadFileName {};
  std::string saveFileName {};
  unsigned short port = 8080;
  size_t max_refinements = 1000;

  unsigned short dim = 3;

  po::options_description loading { "loading options" };
  loading.add_options()("text,t", po::value<std::string>(&loadFileName),
      "load graph from text file")("bin,b", po::value<std::string>(&loadFileName),
      "load graph form binary file")("multi,m", po::value<std::string>(&loadFileName),
      "load graph from multiple files")("zi", "input text file is gzipped")(
      "dimension,d", po::value<unsigned short>(&dim), "Dimension of loaded Graph");

  po::options_description action { "actions" };
  action.add_options()("save", po::value<std::string>(&saveFileName), "save graph to binary file");
  action.add_options()("test", "runs normal dijkstra and CH dijktra for comparison");
  action.add_options()("web,w", "start webserver for interaction via browser");

  po::options_description web { "web options" };
  web.add_options()("port", po::value<unsigned short>(&port), "port to listen on");
  web.add_options()("max-refinements", po::value<size_t>(&max_refinements),
      "Maximal allowed refinement limit for route enumeration");

  po::options_description all;
  all.add_options()("help,h", "prints help message");
  all.add(loading).add(action).add(web);

  po::variables_map vm {};
  po::store(po::parse_command_line(argc, argv, all), vm);
  po::notify(vm);

  if (vm.count("help") > 0) {
    std::cout << all << '\n';
    return 0;
  }
  switch (dim) {
  case 1: {
    return run<1>(vm, loadFileName, saveFileName, port, max_refinements);
    break;
  }
  case 2: {
    return run<2>(vm, loadFileName, saveFileName, port, max_refinements);
    break;
  }
  case 3: {
    return run<3>(vm, loadFileName, saveFileName, port, max_refinements);
    break;
  }
  default:
    std::cout << "Code is not compiled for Dimension " << dim << '\n';
    break;
  }

  return 0;
}
