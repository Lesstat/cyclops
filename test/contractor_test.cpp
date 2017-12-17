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
#include "catch.hpp"
#include "contractor.hpp"
#include "dijkstra.hpp"
#include "graph.hpp"
#include <sstream>

TEST_CASE("Shortcut creation")
{
  Contractor c{};
  Edge e1 = Edge::createFromText("10 15 13.2 5 2 -1 -1");
  Edge e2 = Edge::createFromText("15 22 17.7 2 4 -1 -1");
  SECTION("Creating a shortcut from 2 Edges")
  {

    Edge shortcut = c.createShortcut(e1, e2);
    testEdgeInternals(shortcut, NodeId{ 10 }, NodeId{ 22 },
        Length{ 30.9 }, Height{ 7 }, Unsuitability{ 6 }, e1.getId(), e2.getId());
  }

  SECTION("Shortcut creation fails if edges do not connect on middle node")
  {
    REQUIRE_THROWS_WITH(c.createShortcut(e2, e1), "Edges are not connected");
  }
}

TEST_CASE("Test if edges form shortest path")
{

  std::string file{ R"!!(# Build by: pbfextractor
# Build on: SystemTime { tv_sec: 1512985452, tv_nsec: 881838750 }

3
3
0 163354 48.6674338 9.2445911 380 0
1 163355 48.6694744 9.2432625 380 0
2 163358 48.6661932 9.2515536 386 0
0 1 2.5 7 4 -1 -1
1 2 3.2 9 2 -1 -1
0 2 8.276483027784113 0 2 -1 -1

)!!" };
  auto iss = std::istringstream(file);
  auto g = Graph::createFromStream(iss);
  NodeId nodeId1{ 1 };
  const auto& node1 = g.getNode(nodeId1);
  auto[inEdge, end] = g.getIngoingEdgesOf(nodeId1); //NOLINT
  EdgeId edgeId0 = *inEdge;

  auto[outEdge, outEnd] = g.getOutgoingEdgesOf(nodeId1); //NOLINT
  EdgeId edgeId1 = *outEdge;

  Contractor c{};

  SECTION("With config where the edges form shortest path")
  {
    Config lengthOnlyConf{ LengthConfig{ 1 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } };
    REQUIRE(c.isShortestPath(g, edgeId0, edgeId1, lengthOnlyConf) == true);
  }

  SECTION("With config where the edges do not form shortest path")
  {
    Config heightOnlyConf{ LengthConfig{ 0 }, HeightConfig{ 1 }, UnsuitabilityConfig{ 0 } };
    REQUIRE(c.isShortestPath(g, edgeId0, edgeId1, heightOnlyConf) == false);
  }
}