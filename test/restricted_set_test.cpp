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

TEST_CASE("Exclude route which exceeds second metric slack")
{

  const std::string text_graph { R"!!(# Build by: pbfextractor
# Build on: SystemTime { tv_sec: 1512985452, tv_nsec: 881838750 }

2
8
12
0 163354 48.6674338 9.2445911 380 0
1 163355 48.6694744 9.2432625 380 0
2 163358 48.6661932 9.2515536 386 0
3 163359 48.6261932 9.2515936 386 0
4 163355 48.6694744 9.2432625 380 0
5 163359 48.6261932 9.2515936 386 0
6 163359 48.6261932 9.2515936 386 0
7 163359 48.6261932 9.2515936 386 0
0 1 1 9 -1 -1
1 4 1 9 -1 -1
0 2 9 1 -1 -1
2 4 9 1 -1 -1
0 3 2 2 -1 -1
3 4 2 2 -1 -1
0 5 3 3 -1 -1
5 4 3 3 -1 -1
0 6 3 1.5 -1 -1
6 4 3 1.5 -1 -1
0 7 4 1.25 -1 -1
7 4 4 1.25 -1 -1
)!!" };
  using Graph = Graph<2>;
  using Route = Route<2>;

  std::istringstream stream { text_graph };
  auto g = Graph::createFromStream(stream);

  NodePos s { 0 };
  NodePos t { 4 };

  {
    EnumerateOptimals o { &g, 20, 100 };
    o.find(s, t);
    auto result = o.recommend_routes(true);
    auto routes = std::get<std::vector<Route>>(result);
    REQUIRE(routes.size() == 5);
  }

  {
    Slack<2> slack = { { 2.5, std::numeric_limits<double>::max() } };

    SimilarityPrioPolicy<2> pp(1.0);
    ThresholdPolicy<2> tp(slack);

    EnumerateOptimals<2, ThresholdPolicy<2>, SimilarityPrioPolicy<2>> r { &g, 20, 100, pp, tp };
    r.find(s, t);
    auto result = r.recommend_routes(true);
    auto routes = std::get<std::vector<Route>>(result);
    REQUIRE(routes.size() == 2);
  }
}
