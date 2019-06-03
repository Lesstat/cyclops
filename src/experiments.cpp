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
#include "edge_load.hpp"
#include "enumerate_optimals.hpp"
#include "graph_loading.hpp"
#include "grid.hpp"
#include "ilp_independent_set.hpp"
#include "naive_exploration.hpp"
#include "routeComparator.hpp"
#include "url_parsing.hpp"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <random>

namespace po = boost::program_options;
namespace c = std::chrono;

void printErrorAndHelp(const std::string& error, const po::options_description& all)
{
  std::cout << error << '\n';
  std::cout << all << '\n';
}

template <int Dim>
void explore_random(std::ifstream& file, std::ofstream& output, Dijkstra<Dim>& d, size_t splitCount,
    double maxSimilarity, std::string type)
{
  using Route = Route<Dim>;

  auto counter = 0;
  while (!file.eof()) {
    size_t from;
    size_t to;
    file >> from >> to;
    if (from == to) {
      continue;
    }
    if (file.eof()) {
      break;
    }
    if (++counter % 20 == 0) {
      output.flush();
    }
    try {
      std::vector<Route> routes;
      auto start = c::high_resolution_clock::now();
      for (size_t i = 0; i < splitCount * 3; ++i) {
        routes.push_back(
            d.findBestRoute(NodePos { from }, NodePos { to }, generateRandomConfig<Dim>()).value());
      }
      auto end = c::high_resolution_clock::now();
      size_t exploreTime = c::duration_cast<ms>(end - start).count();

      std::vector<std::pair<size_t, size_t>> edges;
      for (size_t i = 0; i < routes.size(); ++i) {
        for (size_t j = i + 1; j < routes.size(); ++j) {
          if (calculateSharing(routes[i], routes[j]) > maxSimilarity) {
            edges.emplace_back(i, j);
          }
        }
      }
      start = c::high_resolution_clock::now();
      auto ilpSet = find_independent_set(routes.size(), edges);
      end = c::high_resolution_clock::now();
      size_t ilp_recommendation_time = c::duration_cast<ms>(end - start).count();

      duplicate_edges(edges);
      start = c::high_resolution_clock::now();
      auto greedySet = greedy_independent_set(routes.size(), edges);
      end = c::high_resolution_clock::now();
      size_t greedy_recommendation_time = c::duration_cast<ms>(end - start).count();

      output << type << "," << from << "," << to << "," << routes.size() << "," << maxSimilarity
             << "," << routes.size() << "," << ilpSet.size() << "," << greedySet.size() << ","
             << exploreTime << "," << ilp_recommendation_time << "," << greedy_recommendation_time
             << '\n';
    } catch (...) {
      continue;
    }
  }
}

template <int Dim>
void enumerate(std::ifstream& file, std::ofstream& output, Graph<Dim>* g, size_t maxRoutes,
    double maxSimilarity, std::string type)
{
  using Route = Route<Dim>;
  auto d = g->createDijkstra();
  auto counter = 0;
  while (!file.eof()) {
    size_t from;
    size_t to;
    file >> from >> to;
    if (from == to) {
      continue;
    }
    if (file.eof()) {
      break;
    }
    if (++counter % 20 == 0) {
      output.flush();
    }
    std::vector<double> conf(Dim, 0);
    conf[0] = 1;

    if (!d.findBestRoute(NodePos { from }, NodePos { to }, conf)) {
      std::cout << "did not find routes"
                << "\n";
      continue;
    }
    try {
      EnumerateOptimals<Dim, SimilarityPrio> o(g, maxRoutes);
      o.set_overlap(maxSimilarity);

      std::vector<Route> routes;
      o.find(NodePos { from }, NodePos { to });

      std::tie(routes, std::ignore, std::ignore) = o.recommend_routes(true);
      auto routes_recommended_ilp = routes.size();
      auto ilp_time = o.recommendation_time;

      std::tie(routes, std::ignore, std::ignore) = o.recommend_routes(false);
      auto routes_recommended_greedy = routes.size();
      auto greedy_time = o.recommendation_time;

      auto routeCount = o.found_route_count();

      output << type << "," << from << "," << to << "," << maxRoutes << "," << maxSimilarity << ","
             << routeCount << "," << routes_recommended_ilp << "," << routes_recommended_greedy
             << "," << o.enumeration_time << "," << ilp_time << "," << greedy_time << '\n';
    } catch (...) {
      continue;
    }
  }
}

template <int Dim>
void enumerate_all(std::ifstream& file, std::ofstream& output, Graph<Dim>* g, std::string type)
{
  using Route = Route<Dim>;
  size_t from;
  size_t to;
  int counter = 0;
  std::vector<double> conf(Dim, 0);
  conf[0] = 1;
  auto d = g->createDijkstra();
  while (file >> from >> to) {
    if (!d.findBestRoute(NodePos { from }, NodePos { to }, conf)) {
      std::cout << "did not find routes"
                << "\n";
      continue;
    }
    try {
      if (++counter % 20 == 0) {
        std::cout << "finished " << counter << " s-t pairs" << '\n';
        output.flush();
      }
      EnumerateOptimals<Dim, SimilarityPrio> o(g, std::numeric_limits<size_t>::max());

      std::vector<Route> routes;
      o.find(NodePos { from }, NodePos { to });

      std::tie(routes, std::ignore, std::ignore) = o.recommend_routes(true);

      auto routes_recommended = routes.size();

      output << type << "," << from << "," << to << "," << routes_recommended << ","
             << o.enumeration_time << '\n';
    } catch (...) {
      continue;
    }
  }
}

template <int Dim> void search_candidates(Graph<Dim>& g, size_t count)
{
  using Config = Config<Dim>;

  std::cout << "searching for candidates" << '\n';
  std::ofstream candidates;
  candidates.open("candidates", std::ios::app);
  candidates.seekp(0, std::ios_base::end);

  std::random_device rd {};
  std::uniform_int_distribution<size_t> dist(0, g.getNodeCount() - 1);

  auto d = g.createDijkstra();
  std::vector<double> conf(Dim, 0.0);
  conf[0] = 1;
  Config c = conf;

  size_t found_routes = 0;

  while (found_routes < count) {
    if (found_routes % 10 == 0) {
      std::cout << "found " << found_routes << " routes." << '\n';
    }
    NodePos from { dist(rd) };
    NodePos to { dist(rd) };

    auto mayRoute = d.findBestRoute(from, to, c);
    if (!mayRoute) {
      continue;
    }
    found_routes++;

    auto& from_node = g.getNode(from);
    candidates << from_node.lat() << ' ' << from_node.lng() << ' ';
    auto& to_node = g.getNode(to);
    candidates << to_node.lat() << ' ' << to_node.lng() << '\n';
  }
}

template <int Dim> void search_commuting_candidates(Graph<Dim>& g)
{
  using Config = Config<Dim>;
  std::ofstream commute {};
  commute.open("commute", std::ios::app);
  commute.seekp(0, std::ios::end);

  std::vector<NodePos> candidates;

  auto d = g.createDijkstra();
  std::vector<double> values(Dim, 0.0);
  values[0] = 1;
  Config c = values;

  std::random_device rd {};
  std::uniform_int_distribution<size_t> dist(0, g.getNodeCount() - 1);

  auto schlossPos = PositionalNode { Lat { 48.77846 }, Lng { 9.18018 }, NodePos { 1 } };
  for (int i = 0; i < 10000; ++i) {
    NodePos otherPos { dist(rd) };
    auto other = g.getNode(otherPos);

    auto length
        = haversine_distance(schlossPos, PositionalNode { other.lat(), other.lng(), otherPos });
    if (length <= 10000) {
      candidates.push_back(otherPos);
    }
  }

  auto pairs = 0;
  std::cout << "found " << candidates.size() << " candidates" << '\n';
  for (size_t i = 0; i < candidates.size(); ++i) {
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (pairs >= 250) {
        std::cout << "found " << pairs << " pairs"
                  << "\n";
        return;
      }
      auto route = d.findBestRoute(candidates[i], candidates[j], c);
      if (route && route->costs.values[0] > 2000) {
        commute << candidates[i] << " " << candidates[j] << "\n";
        pairs++;
      }
    }
  }
  std::cout << "found " << pairs << " pairs"
            << "\n";
}

template <int Dim>
void explore_naive(std::ifstream& route_input, std::ofstream& output, Dijkstra<Dim>& d,
    double epsilon, double maxSimilarity, std::string type)
{
  int counter = 0;
  for (size_t from, to; route_input >> from >> to;) {
    if (++counter % 20 == 0) {
      output.flush();
    }
    try {
      auto start = c::high_resolution_clock::now();
      auto routes = naiveExploration(d, NodePos { from }, NodePos { to }, epsilon);
      auto end = c::high_resolution_clock::now();
      size_t exploreTime = c::duration_cast<ms>(end - start).count();

      std::vector<std::pair<size_t, size_t>> edges;
      for (size_t i = 0; i < routes.size(); ++i) {
        for (size_t j = i + 1; j < routes.size(); ++j) {
          if (calculateSharing(routes[i], routes[j]) > maxSimilarity) {
            edges.emplace_back(i, j);
          }
        }
      }

      start = c::high_resolution_clock::now();
      auto ilpSet = find_independent_set(routes.size(), edges);
      end = c::high_resolution_clock::now();
      size_t ilp_recommendation_time = c::duration_cast<ms>(end - start).count();

      start = c::high_resolution_clock::now();
      auto greedySet = greedy_independent_set(routes.size(), edges);
      end = c::high_resolution_clock::now();
      size_t greedy_recommendation_time = c::duration_cast<ms>(end - start).count();

      output << type << "," << from << "," << to << "," << routes.size() << "," << maxSimilarity
             << "," << routes.size() << "," << ilpSet.size() << "," << greedySet.size() << ","
             << exploreTime << "," << ilp_recommendation_time << "," << greedy_recommendation_time
             << '\n';
    } catch (...) {
      continue;
    }
  }
}

template <int Dim>
void restricted_exploration(std::ifstream& input, std::ofstream& output, Grid& g, Graph<Dim>* graph,
    const std::string& graphfile, std::string restriction_parameters)
{
  if (output.tellp() == 0) {
    output << "s_lat,s_lng,t_lat,t_lng,inputgraph,starttime,restriction_parameters,all_time,#all_"
              "routes,all_avg_load,all_max_load,restr_time,#restr_routes,restr_avg_load,restr_max_"
              "load\n";
  }

  boost::filesystem::path p(graphfile);
  auto graph_file_name = p.filename();
  auto now = c::system_clock::to_time_t(c::system_clock::now());
  auto starttime = std::localtime(&now);

  const size_t refinements = std::numeric_limits<size_t>::max();
  double s_lat, s_lng, t_lat, t_lng;

  while (input >> s_lat >> s_lng >> t_lat >> t_lng) {
    auto s = g.findNextNode(Lat(s_lat), Lng(s_lng));
    auto t = g.findNextNode(Lat(t_lat), Lng(t_lng));
    if (!(s && t)) {
      std::cerr << "could not find points for " << s_lat << ", " << s_lng << ", " << t_lat << ", "
                << t_lng << '\n';
      continue;
    }

    auto start = c::high_resolution_clock::now();
    EnumerateOptimals<Dim, DefaultsOnly> all(graph, refinements);
    all.find(*s, *t);
    auto result = all.recommend_routes(false);
    auto end = c::high_resolution_clock::now();

    auto all_time = c::duration_cast<ms>(end - start).count();
    auto routes = std::get<std::vector<Route<Dim>>>(result);
    auto all_route_count = routes.size();

    start = c::high_resolution_clock::now();
    EnumerateOptimals<Dim, OnlyExclusion> restricted(graph, refinements);
    Slack<Dim> slack
        = important_metrics_to_array<Dim>(parse_important_metric_list(restriction_parameters));
    restricted.set_slack(slack);
    restricted.find(*s, *t);
    auto result2 = restricted.recommend_routes(false);
    end = c::high_resolution_clock::now();

    auto restricted_time = c::duration_cast<ms>(end - start).count();
    auto routes2 = std::get<std::vector<Route<Dim>>>(result2);
    auto restricted_route_count = routes2.size();

    EdgeLoads all_loads(routes);

    EdgeLoads restr_loads(routes2);

    output << s_lat << ',' << s_lng << ',' << t_lat << ',' << t_lng << ',' << graph_file_name << ','
           << std::put_time(starttime, "%Y-%m-%d %T") << ',' << '"' << restriction_parameters << '"'
           << ',' << all_time << ',' << all_route_count << ',' << all_loads.avg_load() << ','
           << all_loads.max_load() << ',' << restricted_time << ',' << restricted_route_count << ','
           << restr_loads.avg_load() << ',' << restr_loads.max_load() << '\n';
    output.flush();
  }
}

template <int Dim>
void explore_s_t_pairs(std::ifstream& input, std::ofstream& output, Grid& g, Graph<Dim>* graph,
    const std::string& graphfile)
{
  if (output.tellp() == 0) {
    output << "s_lat,s_lng,t_lat,t_lng,inputgraph,starttime,R,K,time,#routes,min_similarity,"
              "max_similarity,avg_similarity\n";
  }
  boost::filesystem::path p(graphfile);
  auto graph_file_name = p.filename();
  auto now = c::system_clock::to_time_t(c::system_clock::now());
  auto starttime = std::localtime(&now);

  const size_t refinements = 40;
  const double max_similarity = 0.5;
  double s_lat, s_lng, t_lat, t_lng;
  EnumerateOptimals<Dim, SimilarityPrio> e(graph, refinements);
  e.set_overlap(max_similarity);
  while (input >> s_lat >> s_lng >> t_lat >> t_lng) {
    auto s = g.findNextNode(Lat(s_lat), Lng(s_lng));
    auto t = g.findNextNode(Lat(t_lat), Lng(t_lng));
    if (!(s && t)) {
      std::cerr << "could not find points for " << s_lat << ", " << s_lng << ", " << t_lat << ", "
                << t_lng << '\n';
      continue;
    }

    auto start = c::high_resolution_clock::now();
    e.find(*s, *t);
    auto [routes, configs, edges] = e.recommend_routes(false);
    auto end = c::high_resolution_clock::now();

    auto time = c::duration_cast<ms>(end - start).count();
    auto route_count = routes.size();

    double min_sim = 1;
    double max_sim = 0;
    double sum_sim = 0;
    double count_sim = 0;
    for (size_t i = 0; i < routes.size(); ++i) {
      for (size_t j = i + 1; j < routes.size(); ++j) {
        double sim = calculateSharing(routes[i], routes[j]);
        if (sim > max_sim)
          max_sim = sim;
        if (sim < min_sim)
          min_sim = sim;
        sum_sim += sim;
        count_sim++;
      }
    }
    double avg_sim = sum_sim / count_sim;
    if (count_sim <= 1) {
      min_sim = 0;
      max_sim = 0;
    }

    output << s_lat << ',' << s_lng << ',' << t_lat << ',' << t_lng << ',' << graph_file_name << ','
           << std::put_time(starttime, "%Y-%m-%d %T") << ',' << refinements << ',' << max_similarity
           << ',' << time << ',' << route_count << ',' << min_sim << ',' << max_sim << ','
           << avg_sim << '\n';
    output.flush();
  }
}

int main(int argc, char* argv[])
{
  std::cout.imbue(std::locale(""));

  std::string loadFileName {};
  std::string saveFileName {};
  size_t candidate_count = 0;

  po::options_description loading { "Graph Loading Options" };
  // clang-format off
  loading.add_options()
    ("text,t", po::value<std::string>(&loadFileName), "load graph from text file")
    ("bin,b", po::value<std::string>(&loadFileName), "load graph form binary file")
    ("zi", "input text file is gzipped");
  // clang-format on

  std::string parameterInputFile {};
  po::options_description dataConfiguration { "Data Configuration options" };
  // clang-format off
  dataConfiguration.add_options()
    ("input,i", po::value<std::string>(&parameterInputFile), "File with parameters to execute on")
    ("output,o", po::value<std::string>(&saveFileName), "File to store results in");
  // clang-format on

  std::string restriction_parameter;
  po::options_description expr { "Experiment to conduct" };
  // clang-format off
  expr.add_options()
    ("dijkstra,d", "Run dijkstra for random s-t queries ")
    ("candidates,c", po::value<size_t>(&candidate_count), "find candidates s-t pairs ")
    ("commute", "find candidates for commuting tours")
    ("random,r", "explore random configurations")
    ("naive,n", "naive exploration")
    ("enumerate,e", "enumerate paths")
    ("all", "enumerate all paths")
    ("st", "Run enumerate on s-t pairs")
    ("restricted,r", po::value<std::string>(&restriction_parameter) , "Compare restricted enumeration to full enumeration") ;
  // clang-format on
  po::options_description all;
  all.add_options()("help,h", "prints help message");
  all.add(loading).add(dataConfiguration).add(expr);

  po::variables_map vm {};
  po::store(po::parse_command_line(argc, argv, all), vm);
  po::notify(vm);

  if (vm.count("help") > 0) {
    std::cout << all << '\n';
    return 0;
  }

  const int Dim = 3;

  Graph<Dim> g { std::vector<Node>(), std::vector<Edge<Dim>>() };
  if (vm.count("text") > 0) {
    bool zipped_input = vm.count("zi") > 0;
    g = loadGraphFromTextFile<Dim>(loadFileName, zipped_input);
  } else if (vm.count("bin") > 0) {
    g = loadGraphFromBinaryFile<Dim>(loadFileName);
  } else {
    printErrorAndHelp("No input graph given", all);
    return 1;
  }

  Dijkstra d = g.createDijkstra();
  std::random_device rd {};
  std::uniform_int_distribution<size_t> dist(0, g.getNodeCount() - 1);
  if (vm.count("candidates") > 0) {
    search_candidates(g, candidate_count);
    return 0;
  } else if (vm.count("commute") > 0) {
    search_commuting_candidates(g);
    return 0;
  }

  std::ifstream params {};
  params.open(parameterInputFile);
  if (!params.good()) {
    printErrorAndHelp("Parameter file " + parameterInputFile + " could not be read", all);
    return 1;
  }

  std::ofstream output {};
  output.open(saveFileName, std::ios::app);
  output.seekp(0, std::ios::end);
  if (!output.good()) {
    printErrorAndHelp("Output file " + parameterInputFile + " could not be read", all);
    return 1;
  }
  if (vm.count("enumerate") > 0) {
    if (output.tellp() == 0) {
      output << "type,from,to,maxRoutes,maxSimilarity,routeCount,ilpRecommendedRouteCount,"
                "greedyRecommendedRouteCount,exploreTime,ilpRecommendationTime,"
                "greedyRecommendationTime\n";
    }
    size_t maxRoutes = 0;
    double maxSimilarity = 1.0;

    while (params >> maxRoutes >> maxSimilarity) {
      std::cout << "Starting Configuration: " << maxRoutes << " " << maxSimilarity << "\n";

      std::ifstream day { "daytour" };
      std::ifstream week { "weektour" };
      std::ifstream commute { "commute" };
      enumerate(day, output, &g, maxRoutes, maxSimilarity, "day");
      enumerate(week, output, &g, maxRoutes, maxSimilarity, "week");
      enumerate(commute, output, &g, maxRoutes, maxSimilarity, "commute");
    }
  } else if (vm.count("all") > 0) {
    if (output.tellp() == 0) {
      output << "type,from,to,routeCount,time\n";
    }
    std::ifstream day { "daytour" };
    std::ifstream week { "weektour" };
    std::ifstream commute { "commute" };
    enumerate_all(commute, output, &g, "commute");
    enumerate_all(day, output, &g, "day");
    // enumerate_all(week, output, d, "week");
  } else if (vm.count("dijkstra") > 0) {
    if (output.tellp() == 0) {
      output << "from,to,";
      for (size_t i = 0; i < Dim; ++i) {
        output << "alpha" << i << ",";
      }
      output << "length,heigh_gain,unsuitabiltiy,edgeCount,pqPolls,time\n";
    }
    auto grid = g.createGrid();
    double lat_f, lng_f, lat_t, lng_t;
    Config<Dim> conf = { { 1.0, 0.0, 0.0 } };

    std::vector<std::pair<NodePos, NodePos>> st_pairs;
    while (params >> lat_f >> lng_f >> lat_t >> lng_t) {
      NodePos from = *grid.findNextNode(Lat { lat_f }, Lng { lng_f });
      NodePos to = *grid.findNextNode(Lat { lat_t }, Lng { lng_t });
      st_pairs.emplace_back(from, to);
    }

    for (auto& [from, to] : st_pairs) {

      // Config conf = generateRandomConfig();
      auto start = std::chrono::high_resolution_clock::now();
      auto optRoute = d.findBestRoute(from, to, conf);
      auto end = std::chrono::high_resolution_clock::now();
      auto time = std::chrono::duration_cast<ms>(end - start).count();

      if (!optRoute) {
        std::cerr << "no route found between " << from << " and " << to << '\n';
        continue;
      }
      auto route = *optRoute;
      output << from << "," << to << ",";
      for (auto& val : conf.values) {
        output << val << ",";
      }
      for (auto& cost : route.costs.values) {
        output << cost << ",";
      }
      output << route.edges.size() << "," << d.pqPops << "," << time << '\n';
    }

  } else if (vm.count("random") > 0) {
    if (output.tellp() == 0) {
      output << "type,from,to,maxRoutes,maxSimilarity,routeCount,ilpRecommendedRouteCount,"
                "greedyRecommendedRouteCount,exploreTime,ilpRecommendationTime,"
                "greedyRecommendationTime\n";
    }
    size_t splitCount = 0;
    double maxSimilarity = 1.0;
    while (params >> splitCount >> maxSimilarity) {

      std::cout << "Starting Configuration: " << splitCount << " " << maxSimilarity << "\n";

      std::ifstream day { "daytour" };
      std::ifstream week { "weektour" };
      std::ifstream commute { "commute" };
      explore_random(day, output, d, splitCount, maxSimilarity, "day");
      explore_random(week, output, d, splitCount, maxSimilarity, "week");
      explore_random(commute, output, d, splitCount, maxSimilarity, "commute");
    }
  } else if (vm.count("naive") > 0) {
    if (output.tellp() == 0) {
      output << "type,from,to,maxRoutes,maxSimilarity,routeCount,ilpRecommendedRouteCount,"
                "greedyRecommendedRouteCount,exploreTime,ilpRecommendationTime,"
                "greedyRecommendationTime\n";
    }

    for (double epsilon, maxSimilarity; params >> epsilon >> maxSimilarity;) {
      std::cout << "running config: " << epsilon << " " << maxSimilarity << '\n';

      std::ifstream day { "daytour" };
      std::ifstream week { "weektour" };
      std::ifstream commute { "commute" };
      explore_naive(commute, output, d, epsilon, maxSimilarity, "commute");
      explore_naive(day, output, d, epsilon, maxSimilarity, "day");
      explore_naive(week, output, d, epsilon, maxSimilarity, "week");
    }
  } else if (vm.count("st") > 0) {
    Grid grid = g.createGrid();
    explore_s_t_pairs(params, output, grid, &g, loadFileName);
  } else if (vm.count("restricted") > 0) {
    Grid grid = g.createGrid();
    restricted_exploration(params, output, grid, &g, loadFileName, restriction_parameter);
  }
  return 0;
}
