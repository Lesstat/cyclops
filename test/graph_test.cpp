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
#include "dijkstra.hpp"
#include "graph.hpp"
#include <sstream>

TEST_CASE("Offset array is correctly initialized")
{
  std::vector<Node> nodes;
  nodes.emplace_back(Node{ NodeId(0), Lat(3.4), Lng(4.6), Height(3) });
  nodes.emplace_back(Node{ NodeId(1), Lat(3.4), Lng(4.6), Height(3) });
  nodes.emplace_back(Node{ NodeId(2), Lat(3.4), Lng(4.6), Height(3) });
  nodes.emplace_back(Node{ NodeId(3), Lat(3.4), Lng(4.6), Height(3) });

  std::vector<Edge> edges;
  edges.emplace_back(Edge{ NodeId(1), NodeId(0) });
  edges.emplace_back(Edge{ NodeId(0), NodeId(2) });
  edges.emplace_back(Edge{ NodeId(0), NodeId(2) });
  edges.emplace_back(Edge{ NodeId(1), NodeId(2) });

  Graph g{ std::move(nodes), std::move(edges) };

  auto offsets = g.getOffsets();

  REQUIRE(offsets[0].out == 0);
  REQUIRE(offsets[1].out == 2);
  REQUIRE(offsets[2].out == 4);
  REQUIRE(offsets[3].out == 4);
  REQUIRE(offsets[4].out == 4);

  REQUIRE(offsets[0].in == 0);
  REQUIRE(offsets[1].in == 1);
  REQUIRE(offsets[2].in == 1);
  REQUIRE(offsets[3].in == 4);
  REQUIRE(offsets[4].in == 4);
}

TEST_CASE("Read small file into graph")
{
  std::string file{ R"!!(# Type : chgraph
# Id : f5c398be8e451b8fe2b170dca6a87563
# Revision : 1
# Timestamp : 1493032504
# Origin : ch_constructor
# OriginId : 0
# OriginRevision : 1
# OriginTimestamp : 1491925284
# OriginType : maxspeed

3
2
0 470552 49.3413737 7.3014905 0 0
1 470553 49.3407609 7.3007752 0 0
2 470554 49.3405748 7.3002951 0 0
0 1 85 2 70 -1 -1
1 2 16 2 70 -1 -1

)!!" };
  auto iss = std::istringstream(file);
  Graph g = Graph::createFromStream(iss);

  auto dij = g.createDijkstra();
  auto optionalRoute = dij.findBestRoute(NodePos{ 0 }, NodePos{ 2 },
      Config{ LengthConfig{ 1.0 }, HeightConfig{ 0 }, UnsuitabilityConfig{ 0 } });

  REQUIRE(optionalRoute.has_value());
  auto route = optionalRoute.value();
  REQUIRE(route.edges[0].getSourceId() == 0);
  REQUIRE(route.edges[1].getSourceId() == 1);
  REQUIRE(route.costs.length == 101);
  REQUIRE(route.costs.height == 4);
  REQUIRE(route.costs.unsuitability == 140);
}
