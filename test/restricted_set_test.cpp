/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2019  Florian Barth

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
#include "dijkstra.hpp"
#include "enumerate_optimals.hpp"
#include "graph.hpp"

TEST_CASE("Restricted Dijkstra does not choose long path")
{

  const std::string threeNodeGraph {
    R"!!(# Build by: pbfextractor
# Build on: SystemTime { tv_sec: 1512985452, tv_nsec: 881838750 }

3
3
3
0 163354 48.6674338 9.2445911 380 0
1 163355 48.6694744 9.2432625 380 0
2 163358 48.6661932 9.2515536 386 0
0 2 0.0 2 12 -1 -1
0 1 0.0 8 3  -1 -1
1 2 0.0 8 3  -1 -1)!!"
  };

  std::istringstream iss(threeNodeGraph);
  Graph g = Graph::createFromStream(iss);

  auto d = g.createDijkstra();
  const NodePos s { 0 };
  const NodePos t { 2 };

  Config a = std::vector<double> { 0, 1, 0 };
  auto a_route = d.findBestRoute(s, t, a);
  REQUIRE(a_route->edges.size() == 1);
  exclusion_set excluded(3, false);
  d.excluded_nodes(1.1, excluded);
  exclusion_set expected_exclusions = { false, true, false };
  for (size_t i = 0; i < excluded.size(); ++i)
    REQUIRE(excluded[i] == expected_exclusions[i]);

  Config b = std::vector<double> { 0, 0, 1 };
  auto b_route = d.findBestRoute(s, t, b);
  REQUIRE(b_route->edges.size() == 2);

  d.exclude(excluded);
  b_route = d.findBestRoute(s, t, b);
  REQUIRE(b_route->edges.size() == 1);
}

TEST_CASE("Exclude route which exceeds second metric slack")
{

  const std::string sixNodeGraph { R"!!(# Build by: pbfextractor
# Build on: SystemTime { tv_sec: 1512985452, tv_nsec: 881838750 }

3
6
8
0 163354 48.6674338 9.2445911 380 0
1 163355 48.6694744 9.2432625 380 0
2 163358 48.6661932 9.2515536 386 0
3 163359 48.6261932 9.2515936 386 0
4 163355 48.6694744 9.2432625 380 0
5 163359 48.6261932 9.2515936 386 0
0 1 0.0 1 9 -1 -1
1 4 0.0 1 9 -1 -1
0 2 0.0 9 1 -1 -1
2 4 0.0 9 1 -1 -1
0 3 0.0 2 2 -1 -1
3 4 0.0 2 2 -1 -1
0 5 0.0 3 3 -1 -1
5 4 0.0 3 3 -1 -1
)!!" };

  std::istringstream stream { sixNodeGraph };
  auto g = Graph::createFromStream(stream);

  NodePos s { 0 };
  NodePos t { 4 };

  {
    EnumerateOptimals o { &g, 20, 100 };
    o.find(s, t);
    auto result = o.recommend_routes(true);
    auto routes = std::get<std::vector<Route>>(result);

    REQUIRE(routes.size() == 3);
  }

  {
    Important important = { false, true, false };
    Slack slack = { 0, 2.5, 0 };
    EnumerateOptimals r { &g, 20, 100, important, slack };
    r.find(s, t);
    auto result = r.recommend_routes(true);
    auto routes = std::get<std::vector<Route>>(result);
    REQUIRE(routes.size() == 2);
  }
}
