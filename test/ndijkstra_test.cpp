/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2018  Florian Barth

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
#include "catch.hpp"
#include "graph.hpp"
#include "ndijkstra.hpp"
#include <sstream>

TEST_CASE("Find other route with same costs")
{

  const std::string threeNodeGraph{ R"!!(# Build by: pbfextractor
# Build on: SystemTime { tv_sec: 1512985452, tv_nsec: 881838750 }

3
3
0 163354 48.6674338 9.2445911 380 0
1 163355 48.6694744 9.2432625 380 0
2 163358 48.6661932 9.2515536 386 0
0 1 0.0 3 1 -1 -1
1 2 0.0 5 1 -1 -1
0 2 0.0 8 1 -1 -1

)!!" };

  std::istringstream iss(threeNodeGraph);
  Graph g = Graph::createFromStream(iss);

  NormalDijkstra d = g.createNormalDijkstra();

  Config config{ LengthConfig{ 0.0 }, HeightConfig{ 1.0 }, UnsuitabilityConfig{ 0.0 } };
  auto route = d.findBestRoute(NodePos{ 0 }, NodePos{ 2 }, config);
  REQUIRE(route->pathCount == 2);
  REQUIRE(route->edges.size() == 1);

  auto otherRoute = d.findOtherRoute(*route);
  REQUIRE(otherRoute.pathCount == 2);
  REQUIRE(otherRoute.edges.size() == 2);

  auto allRoutes = d.findAllBestRoutes(NodePos{ 0 }, NodePos{ 2 }, 100);
  REQUIRE(allRoutes.size() == 2);
}
