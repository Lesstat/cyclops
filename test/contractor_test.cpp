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
    testEdgeInternals(shortcut, NodeId{ 10 }, NodeId{ 22 }, Length{ 30.9 }, Height{ 7 },
        Unsuitability{ 6 }, e1.getId(), e2.getId());
  }

  SECTION("Shortcut creation fails if edges do not connect on middle node")
  {
    REQUIRE_THROWS_WITH(c.createShortcut(e2, e1), "Edges are not connected");
  }
}

const std::string threeNodeGraph{ R"!!(# Build by: pbfextractor
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
auto iss = std::istringstream(threeNodeGraph);
auto g = Graph::createFromStream(iss);

const std::string fourNodeGraph{ R"!!(# Build by: pbfextractor
# Build on: SystemTime { tv_sec: 1512985452, tv_nsec: 881838750 }

4
3
0 163354 48.6674338 9.2445911 380 0
1 163355 48.6694744 9.2432625 380 0
2 163358 48.6661932 9.2515536 386 0
3 163359 48.6661232 9.2515436 384 0
0 1 2.5 7 4 -1 -1
1 2 3.2 9 2 -1 -1
2 3 8.2 4 2 -1 -1

)!!" };

TEST_CASE("Test if edges form shortest path")
{

  NodePos nodePos1{ 1 };
  const auto& inEdges = g.getIngoingEdgesOf(nodePos1);
  EdgeId edgeId0 = *inEdges.begin();

  const auto& outEdges = g.getOutgoingEdgesOf(nodePos1);
  EdgeId edgeId1 = *outEdges.begin();

  Contractor c{};

  SECTION("With config where the edges form shortest path")
  {
    Config lengthOnlyConf{ LengthConfig{ 1 }, HeightConfig{ 0 },
      UnsuitabilityConfig{ 0 } };
    REQUIRE(c.isShortestPath(g, edgeId0, edgeId1, lengthOnlyConf) == true);
  }

  SECTION("With config where the edges do not form shortest path")
  {
    Config heightOnlyConf{ LengthConfig{ 0 }, HeightConfig{ 1 },
      UnsuitabilityConfig{ 0 } };
    REQUIRE(c.isShortestPath(g, edgeId0, edgeId1, heightOnlyConf) == false);
  }
}

TEST_CASE("Contracting a Node")
{

  Contractor c{};

  SECTION("Where no shortcuts need to be created")
  {
    auto shortcuts = c.contract(g, NodePos{ 2 });
    REQUIRE(shortcuts.empty());
  }
  SECTION("Where the right configuration needs to be found")
  {
    NodePos nodePos1{ 1 };
    const auto& inEdges = g.getIngoingEdgesOf(nodePos1);
    EdgeId edgeId0 = *inEdges.begin();

    const auto& outEdges = g.getOutgoingEdgesOf(nodePos1);
    EdgeId edgeId1 = *outEdges.begin();

    auto shortcuts = c.contract(g, nodePos1);
    REQUIRE(shortcuts.size() == 1);
    testEdgeInternals(shortcuts[0], NodeId{ 0 }, NodeId{ 2 }, Length{ 5.7 },
        Height{ 16 }, Unsuitability{ 6 }, edgeId0, edgeId1);
  }
}
TEST_CASE("Detect cycles when contracting a Node")
{

  const std::string cycleGraph{ R"!!(# Build by: pbfextractor
# Build on: SystemTime { tv_sec: 1512985452, tv_nsec: 881838750 }

2
2
0 163354 48.6674338 9.2445911 380 0
1 163355 48.6694744 9.2432625 380 0
0 1 2.5 0 4 -1 -1
1 0 2.5 0 4 -1 -1
)!!" };
  auto iss = std::istringstream(cycleGraph);
  auto g = Graph::createFromStream(iss);

  Contractor c{};

  auto shortcuts = c.contract(g, NodePos{ 0 });

  REQUIRE(shortcuts.size() == 0);
}

TEST_CASE("Finding and reducing independent sets")
{
  Contractor c{};

  SECTION("In a three Node Graph")
  {
    auto nodeSetWithOneNode = c.independentSet(g);
    REQUIRE(nodeSetWithOneNode.size() == 1);

    auto stillOneNode = c.reduce(nodeSetWithOneNode, g);
    REQUIRE(stillOneNode.size() == 1);
  }

  SECTION("In a four node graph")
  {
    auto iss = std::istringstream(fourNodeGraph);
    auto fourNodeG = Graph::createFromStream(iss);

    auto nodeSetWithTwoNodes = c.independentSet(fourNodeG);
    REQUIRE(nodeSetWithTwoNodes.size() == 2);

    auto onlyOneNode = c.reduce(nodeSetWithTwoNodes, fourNodeG);
    REQUIRE(onlyOneNode.size() == 1);
  }
}

Route findRouteBetweenIds(Graph& g, NodeId from, NodeId to)
{
  Config c{ LengthConfig{ 1 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } };
  NodePos id1Pos = g.nodePosById(from).value();
  NodePos id3Pos = g.nodePosById(to).value();

  auto dijkstra = g.createDijkstra();
  return dijkstra.findBestRoute(id1Pos, id3Pos, c).value();
}

void compareRoutes(Route& routeA, Route& routeB)
{
  REQUIRE(routeA.costs.length == routeB.costs.length);
  REQUIRE(routeA.costs.height == routeB.costs.height);
  REQUIRE(routeA.costs.unsuitability == routeB.costs.unsuitability);
}

TEST_CASE("Contracting one level of Graph")
{
  Contractor c{};

  const std::string fourNodeGraph{ R"!!(# Build by: pbfextractor
# Build on: SystemTime { tv_sec: 1512985452, tv_nsec: 881838750 }

4
4
0 163354 48.6674338 9.2445911 380 0
1 163355 48.6694744 9.2432625 380 0
2 163358 48.6661932 9.2515536 386 0
3 163359 48.6661232 9.2515436 384 0
0 1 2.5 7 4 -1 -1
1 2 3.2 9 2 -1 -1
2 3 8.2 4 2 -1 -1
1 3 99.9 99 99 -1 -1

)!!" };
  auto iss = std::istringstream(fourNodeGraph);
  Graph initialG = Graph::createFromStream(iss);

  Graph intermedG = c.contract(initialG);

  SECTION("New Graph has correct set of nodes")
  {
    REQUIRE(intermedG.getNodeCount() == 2);
    REQUIRE(intermedG.getNode(NodePos{ 0 }).id() == 1);
    REQUIRE(intermedG.getNode(NodePos{ 1 }).id() == 3);
    REQUIRE(intermedG.getEdgeCount() == 2);
  }

  auto finalG = c.mergeWithContracted(intermedG);

  SECTION("Merging Graph with previously contracted nodes")
  {
    REQUIRE(finalG.getNodeCount() == 4);
    REQUIRE(finalG.getNode(NodePos{ 2 }).id() == 0);
    REQUIRE(finalG.getNode(NodePos{ 2 }).getLevel() == 1);
    REQUIRE(finalG.getNode(NodePos{ 3 }).id() == 2);
    REQUIRE(finalG.getNode(NodePos{ 3 }).getLevel() == 1);
    REQUIRE(finalG.getEdgeCount() == 5);
  }

  SECTION("Distances stay the same in all intermediate steps")
  {

    auto routeInitialGraph = findRouteBetweenIds(initialG, NodeId{ 1 }, NodeId{ 3 });
    auto routeIntermedGraph = findRouteBetweenIds(intermedG, NodeId{ 1 }, NodeId{ 3 });
    auto routeFinalGraph = findRouteBetweenIds(finalG, NodeId{ 1 }, NodeId{ 3 });

    compareRoutes(routeInitialGraph, routeIntermedGraph);
    compareRoutes(routeIntermedGraph, routeFinalGraph);
  }
}

TEST_CASE("Fully contract graph")
{
  auto iss = std::istringstream(fourNodeGraph);
  Graph initialG = Graph::createFromStream(iss);
  Contractor c{};

  Graph ch = c.contractCompletely(initialG);

  REQUIRE(ch.getLevelOf(NodePos{ 0 }) == 1);
  REQUIRE(ch.getLevelOf(NodePos{ 1 }) == 1);
  REQUIRE(ch.getLevelOf(NodePos{ 2 }) == 2);
  REQUIRE(ch.getLevelOf(NodePos{ 3 }) == 3);
}
