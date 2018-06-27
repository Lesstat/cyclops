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
#include "multiqueue.hpp"

Graph graphFromString(const std::string& s)
{
  auto iss = std::istringstream(s);
  return Graph::createFromStream(iss);
}

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

  auto g = graphFromString(threeNodeGraph);
  NodePos nodePos1{ 1 };
  const auto& inEdges = g.getIngoingEdgesOf(nodePos1);
  auto& edge0 = *inEdges.begin();

  const auto& outEdges = g.getOutgoingEdgesOf(nodePos1);
  auto& edge1 = *outEdges.begin();

  Contractor c{};
  auto d = g.createNormalDijkstra();

  SECTION("With config where the edges form shortest path")
  {
    Config lengthOnlyConf{ LengthConfig{ 1 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } };
    REQUIRE(c.isShortestPath(d, edge0, edge1, lengthOnlyConf).first == true);
  }

  SECTION("With config where the edges do not form shortest path")
  {
    Config heightOnlyConf{ LengthConfig{ 0 }, HeightConfig{ 1 }, UnsuitabilityConfig{ 0 } };
    REQUIRE(c.isShortestPath(d, edge0, edge1, heightOnlyConf).first == false);
  }
}

std::vector<Edge> contractNode(Graph& g, NodePos pos, Contractor& c)
{
  MultiQueue to{};
  auto back = std::make_shared<MultiQueue>();
  for (const auto& in : g.getIngoingEdgesOf(pos)) {
    for (const auto& out : g.getOutgoingEdgesOf(pos)) {
      to.send(EdgePair{ in, out });
    }
  }
  to.send(back);
  c.contract(to, g);
  std::any msg = back->receive();

  return std::any_cast<std::vector<Edge>>(msg);
}

TEST_CASE("Contracting a Node")
{

  auto g = graphFromString(threeNodeGraph);
  Contractor c{};

  SECTION("Where no shortcuts need to be created")
  {
    auto shortcuts = contractNode(g, NodePos{ 2 }, c);
    REQUIRE(shortcuts.empty());
  }
  SECTION("Where the right configuration needs to be found")
  {
    NodePos nodePos1{ 1 };
    const auto& inEdges = g.getIngoingEdgesOf(nodePos1);
    auto& edge0 = *inEdges.begin();

    const auto& outEdges = g.getOutgoingEdgesOf(nodePos1);
    auto& edge1 = *outEdges.begin();

    auto shortcuts = contractNode(g, nodePos1, c);
    REQUIRE(shortcuts.size() == 1);
    testEdgeInternals(shortcuts[0], NodeId{ 0 }, NodeId{ 2 }, Length{ 5.7 }, Height{ 16 },
        Unsuitability{ 6 }, edge0.id, edge1.id);
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
  auto g = graphFromString(cycleGraph);

  Contractor c{};

  auto shortcuts = contractNode(g, NodePos{ 0 }, c);

  REQUIRE(shortcuts.empty());
}

TEST_CASE("Finding and reducing independent sets")
{
  auto g = graphFromString(threeNodeGraph);
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
  for (size_t i = 0; i < routeA.costs.values.size(); ++i) {
    REQUIRE(routeA.costs.values[i] == routeB.costs.values[i]);
  }
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
  Graph initialG = graphFromString(fourNodeGraph);
  auto routeInitialGraph = findRouteBetweenIds(initialG, NodeId{ 1 }, NodeId{ 3 });

  Graph intermedG = c.contract(initialG);
  auto routeIntermedGraph = findRouteBetweenIds(intermedG, NodeId{ 1 }, NodeId{ 3 });

  SECTION("New Graph has correct set of nodes")
  {
    REQUIRE(intermedG.getNodeCount() == 3);
    REQUIRE(intermedG.getNode(NodePos{ 0 }).id() == 1);
    REQUIRE(intermedG.getNode(NodePos{ 1 }).id() == 2);
    REQUIRE(intermedG.getNode(NodePos{ 2 }).id() == 3);
    REQUIRE(intermedG.getEdgeCount() == 3);
  }

  auto finalG = c.mergeWithContracted(intermedG);

  SECTION("Merging Graph with previously contracted nodes")
  {
    REQUIRE(finalG.getNodeCount() == 4);
    REQUIRE(finalG.getNode(NodePos{ 0 }).id() == 0);
    REQUIRE(finalG.getNode(NodePos{ 0 }).getLevel() == 1);
  }

  SECTION("Distances stay the same in all intermediate steps")
  {

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
  REQUIRE(ch.getLevelOf(NodePos{ 1 }) == 2);
  REQUIRE(ch.getLevelOf(NodePos{ 2 }) == 3);
  REQUIRE(ch.getLevelOf(NodePos{ 3 }) == 4);
}

TEST_CASE("Give max level to not contracted nodes")
{

  const std::string tenNodeGraph{ R"!!(# Build by: pbfextractor
# Build on: SystemTime { tv_sec: 1512985452, tv_nsec: 881838750 }

10
9
0 163354 48.6674338 9.2445911 380 0
1 163355 48.6694744 9.2432625 380 0
2 163358 48.6661932 9.2515536 386 0
3 163354 48.6674338 9.2445911 380 0
4 163355 48.6694744 9.2432625 380 0
5 163358 48.6661932 9.2515536 386 0
6 163354 48.6674338 9.2445911 380 0
7 163355 48.6694744 9.2432625 380 0
8 163358 48.6661932 9.2515536 386 0
9 163358 48.6661932 9.2515536 386 0
0 1 2.5 7 4 -1 -1
1 2 3.2 9 2 -1 -1
2 3 8.2 0 2 -1 -1
3 4 2.5 7 4 -1 -1
4 5 3.2 9 2 -1 -1
5 6 8.2 0 2 -1 -1
6 7 2.5 7 4 -1 -1
7 8 3.2 9 2 -1 -1
8 9 8.2 0 2 -1 -1

)!!" };
  auto iss = std::istringstream(tenNodeGraph);
  auto g = Graph::createFromStream(iss);

  Contractor c{};

  auto ch = c.contractCompletely(g);

  REQUIRE(ch.getLevelOf(NodePos{ 0 }) == 1);
  REQUIRE(ch.getLevelOf(NodePos{ 1 }) == 1);
  REQUIRE(ch.getLevelOf(NodePos{ 2 }) == 2);
  REQUIRE(ch.getLevelOf(NodePos{ 3 }) == 2);
  REQUIRE(ch.getLevelOf(NodePos{ 4 }) == 3);
  REQUIRE(ch.getLevelOf(NodePos{ 5 }) == 4);
  REQUIRE(ch.getLevelOf(NodePos{ 6 }) == 5);
  REQUIRE(ch.getLevelOf(NodePos{ 7 }) == 6);
  REQUIRE(ch.getLevelOf(NodePos{ 8 }) == 7);
  REQUIRE(ch.getLevelOf(NodePos{ 9 }) == 8);
}
