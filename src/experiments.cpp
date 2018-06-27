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
#include "graph_loading.hpp"
#include "grid.hpp"
#include "ilp_independent_set.hpp"
#include "routeComparator.hpp"
#include "scaling_triangulation.hpp"
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

void explore(std::ifstream& file, std::ofstream& output, Dijkstra& d, size_t splitCount,
    size_t maxLevel, double maxSimilarity, std::string type)
{
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
    if (!d.findBestRoute(NodePos{ from }, NodePos{ to }, PosVector{ { 1, 0, 0 } })) {
      std::cout << "did not find routes"
                << "\n";
      continue;
    }
    try {
      auto exploration = scaledTriangulation(d, NodePos{ from }, NodePos{ to }, splitCount,
          maxLevel > 0 ? maxLevel : std::optional<size_t>{}, false, maxSimilarity);

      auto routes_recommended = std::count_if(
          exploration.points.begin(), exploration.points.end(), [](auto p) { return p.selected; });
      auto routeCount = exploration.points.size();

      output << type << "," << from << "," << to << "," << splitCount << "," << maxLevel << ","
             << maxSimilarity << "," << routeCount << "," << routes_recommended << ","
             << exploration.explore_time << "," << exploration.recommendation_time << '\n';
    } catch (...) {
      continue;
    }
  }
}
void explore_random(std::ifstream& file, std::ofstream& output, Dijkstra& d, size_t splitCount,
    double maxSimilarity, std::string type)
{
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
    try {
      std::vector<Route> routes;
      auto start = c::high_resolution_clock::now();
      for (size_t i = 0; i < splitCount * 3; ++i) {
        routes.push_back(
            d.findBestRoute(NodePos{ from }, NodePos{ to }, generateRandomConfig()).value());
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
      auto set = find_independent_set(routes.size(), edges);
      end = c::high_resolution_clock::now();
      size_t recommendation_time = c::duration_cast<ms>(end - start).count();

      output << type << "," << from << "," << to << "," << splitCount << "," << maxSimilarity << ","
             << routes.size() << "," << set.size() << "," << exploreTime << ","
             << recommendation_time << '\n';
    } catch (...) {
      continue;
    }
  }
}

void search_candidates(Graph& g)
{
  std::cout << "searching for candidates" << '\n';
  std::ofstream day;
  day.open("daytour", std::ios::app);
  day.seekp(0, std::ios_base::end);
  std::ofstream week;
  week.open("weektour", std::ios::app);
  week.seekp(0, std::ios_base::end);

  size_t daycount = 0;
  size_t weekcount = 0;

  const double minDay = 40000;
  const double maxDay = 80000;
  const double minWeek = 120000;

  std::random_device rd{};
  std::uniform_int_distribution<size_t> dist(0, g.getNodeCount() - 1);

  auto d = g.createDijkstra();
  Config c = PosVector{ { 1, 0, 0 } };

  for (size_t i = 0; i < 1000; ++i) {
    if (i % 100 == 0) {
      std::cout << "found " << daycount << " daytours and " << weekcount << " more day tours."
                << "\n";
    }
    NodePos from{ dist(rd) };
    NodePos to{ dist(rd) };

    auto mayRoute = d.findBestRoute(from, to, c);
    if (!mayRoute) {
      continue;
    }
    auto route = *mayRoute;
    if (minDay <= route.costs.values[0] && route.costs.values[0] <= maxDay) {
      daycount++;
      day << from.get() << " " << to.get() << '\n';
    } else if (route.costs.values[0] >= minWeek) {
      weekcount++;
      week << from.get() << " " << to.get() << '\n';
    }
  }
}

void search_commuting_candidates(Graph& g)
{

  std::ofstream commute{};
  commute.open("commute", std::ios::app);
  commute.seekp(0, std::ios::end);

  std::vector<NodePos> candidates;

  auto d = g.createDijkstra();
  Config c = PosVector{ { 1, 0, 0 } };

  std::random_device rd{};
  std::uniform_int_distribution<size_t> dist(0, g.getNodeCount() - 1);

  auto schlossPos = PositionalNode{ Lat{ 48.77846 }, Lng{ 9.18018 }, NodePos{ 1 } };
  for (int i = 0; i < 10000; ++i) {
    NodePos otherPos{ dist(rd) };
    auto other = g.getNode(otherPos);

    auto length
        = haversine_distance(schlossPos, PositionalNode{ other.lat(), other.lng(), otherPos });
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

int main(int argc, char* argv[])
{
  std::cout.imbue(std::locale(""));

  std::string loadFileName{};
  std::string saveFileName{};

  po::options_description loading{ "Graph Loading Options" };
  loading.add_options()(
      "text,t", po::value<std::string>(&loadFileName), "load graph from text file")(
      "bin,b", po::value<std::string>(&loadFileName), "load graph form binary file")(
      "multi,m", po::value<std::string>(&loadFileName), "load graph from multiple files");

  std::string parameterInputFile{};
  po::options_description dataConfiguration{ "Data Configuration options" };
  dataConfiguration.add_options()(
      "input,i", po::value<std::string>(&parameterInputFile), "File with parameters to execute on")(
      "output,o", po::value<std::string>(&saveFileName), "File to store results in");

  po::options_description experiment{ "Experiment to conduct" };
  dataConfiguration.add_options()("dijkstra,d", "Run dijkstra for random s-t queries ")(
      "alt,a", "Run alternative route search for random s-t queries")(
      "candidates,c", "find candidates for one and more day tours")("commute",
      "find candidates for commuting tours")("random,r", "explore random configurations");

  po::options_description all;
  all.add_options()("help,h", "prints help message");
  all.add(loading).add(dataConfiguration).add(experiment);

  po::variables_map vm{};
  po::store(po::parse_command_line(argc, argv, all), vm);
  po::notify(vm);

  if (vm.count("help") > 0) {
    std::cout << all << '\n';
    return 0;
  }

  std::ifstream params{};
  params.open(parameterInputFile);
  if (!params.good()) {
    printErrorAndHelp("Parameter file " + parameterInputFile + " could not be read", all);
    return 1;
  }

  std::ofstream output{};
  output.open(saveFileName, std::ios::app);
  output.seekp(0, std::ios::end);
  if (!output.good()) {
    printErrorAndHelp("Output file " + parameterInputFile + " could not be read", all);
    return 1;
  }

  Graph g{ std::vector<Node>(), std::vector<Edge>() };
  if (vm.count("text") > 0) {
    g = loadGraphFromTextFile(loadFileName);
  } else if (vm.count("bin") > 0) {
    g = loadGraphFromBinaryFile(loadFileName);
  } else if (vm.count("multi") > 0) {
    g = readMultiFileGraph(loadFileName);
  } else {
    printErrorAndHelp("No input graph given", all);
    return 1;
  }

  Dijkstra d = g.createDijkstra();
  std::random_device rd{};
  std::uniform_int_distribution<size_t> dist(0, g.getNodeCount() - 1);

  if (vm.count("alt") > 0) {
    // output << "type,from,to,maxSplits,maxLevel,maxSimilarity,routeCount,recommendedRouteCount,"
    //           "exploreTime,recommendationTime\n";
    while (!params.eof()) {
      size_t splitCount = 0, maxLevel = 0;
      double maxSimilarity = 1.0;
      params >> splitCount >> maxLevel >> maxSimilarity;
      std::cout << "Starting Configuration: " << splitCount << " " << maxLevel << " "
                << maxSimilarity << "\n";

      std::ifstream day{ "daytour" };
      std::ifstream week{ "weektour" };
      std::ifstream commute{ "commute" };
      explore(day, output, d, splitCount, maxLevel, maxSimilarity, "day");
      explore(week, output, d, splitCount, maxLevel, maxSimilarity, "week");
      explore(commute, output, d, splitCount, maxLevel, maxSimilarity, "commute");
    }
  } else if (vm.count("dijkstra") > 0) {
    output
        << "from,to,alpha1,alpha2,alpha3,length,heigh_gain,unsuitabiltiy,edgeCount,pqPolls,time\n";
    while (!params.eof()) {
      size_t sampleSize = 0;
      params >> sampleSize;
      std::cout << "Computing " << sampleSize << " dijkstra runs." << '\n';

      size_t curRoute = 0;
      while (curRoute < sampleSize) {
        NodePos from{ dist(rd) };
        NodePos to{ dist(rd) };
        Config conf = generateRandomConfig();
        auto start = std::chrono::high_resolution_clock::now();
        auto optRoute = d.findBestRoute(from, to, conf);
        auto end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<ms>(end - start).count();

        if (!optRoute) {
          continue;
        }
        auto route = *optRoute;
        output << from << "," << to << "," << conf.length << "," << conf.height << ","
               << conf.unsuitability << "," << route.costs.values[0] << "," << route.costs.values[1]
               << "," << route.costs.values[2] << "," << route.edges.size() << "," << d.pqPops
               << "," << time << '\n';
        curRoute++;
      }
    }
  } else if (vm.count("candidates") > 0) {
    search_candidates(g);
  } else if (vm.count("commute") > 0) {
    search_commuting_candidates(g);
  } else if (vm.count("random") > 0) {
    output << "type,from,to,maxSplits,maxSimilarity,routeCount,recommendedRouteCount,"
              "exploreTime,recommendationTime\n";
    while (!params.eof()) {
      size_t splitCount = 0;
      double maxSimilarity = 1.0;
      params >> splitCount >> maxSimilarity;
      std::cout << "Starting Configuration: " << splitCount << " " << maxSimilarity << "\n";

      std::ifstream day{ "daytour" };
      std::ifstream week{ "weektour" };
      std::ifstream commute{ "commute" };
      explore_random(day, output, d, splitCount, maxSimilarity, "day");
      explore_random(week, output, d, splitCount, maxSimilarity, "week");
      explore_random(commute, output, d, splitCount, maxSimilarity, "commute");
    }
  }
  return 0;
}
