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
#include "routeComparator.hpp"
#include "scaling_triangulation.hpp"
#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <random>

namespace po = boost::program_options;

void printErrorAndHelp(const std::string& error, const po::options_description& all)
{
  std::cout << error << '\n';
  std::cout << all << '\n';
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
      "alt,a", "Run alternative route search for random s-t queries");

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
  output.open(saveFileName);
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
    output << "from,to,maxSplits,maxLevel,maxSimilarity,routeCount,time\n";
    while (!params.eof()) {
      size_t splitCount = 0, maxLevel = 0, sampleSize = 0;
      double maxSimilarity = 1.0;
      params >> sampleSize >> splitCount >> maxLevel >> maxSimilarity;
      std::cout << "Starting Configuration: " << sampleSize << " " << splitCount << " " << maxLevel
                << " " << maxSimilarity << "\n";

      size_t curRoute = 0;
      while (curRoute < sampleSize) {
        NodePos from{ dist(rd) };
        NodePos to{ dist(rd) };
        if (!d.findBestRoute(from, to, PosVector({ 1, 0, 0 }))) {
          continue;
        }
        auto start = std::chrono::high_resolution_clock::now();
        auto triPoints = std::get<std::vector<TriPoint>>(
            scaledTriangulation(d, from, to, splitCount, maxLevel, false));
        auto end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<ms>(end - start).count();

        std::vector<bool> uniqueRoute(triPoints.size(), true);

        for (size_t i = 0; i < triPoints.size(); ++i) {
          if (!uniqueRoute[i])
            continue;
          for (size_t j = i + 1; j < triPoints.size(); ++j) {
            auto sharing = calculateSharing(triPoints[i].r, triPoints[j].r);
            if (sharing > maxSimilarity) {
              uniqueRoute[j] = false;
            }
          }
        }
        auto routeCount = std::count(uniqueRoute.begin(), uniqueRoute.end(), true);

        output << from << "," << to << "," << splitCount << "," << maxLevel << "," << maxSimilarity
               << "," << routeCount << "," << time << '\n';
        curRoute++;
      }
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
               << conf.unsuitability << "," << route.costs.length << "," << route.costs.height
               << "," << route.costs.unsuitability << "," << route.edges.size() << "," << d.pqPops
               << "," << time << '\n';
        curRoute++;
      }
    }
  }

  return 0;
}
