
#include "catch.hpp"
#include "graph.hpp"

TEST_CASE("Offset array is correctly initialized")
{
  std::vector<Node> nodes;
  nodes.emplace_back(Node{ OsmId(1), Lat(3.4), Lng(4.6), Height(3.4) });
  nodes.emplace_back(Node{ OsmId(2), Lat(3.4), Lng(4.6), Height(3.4) });
  nodes.emplace_back(Node{ OsmId(3), Lat(3.4), Lng(4.6), Height(3.4) });
  nodes.emplace_back(Node{ OsmId(4), Lat(3.4), Lng(4.6), Height(3.4) });

  std::vector<Edge> edges;
  edges.emplace_back(Edge{ OsmId(7), NodeId(2), NodeId(1) });
  edges.emplace_back(Edge{ OsmId(4), NodeId(1), NodeId(3) });
  edges.emplace_back(Edge{ OsmId(5), NodeId(1), NodeId(3) });
  edges.emplace_back(Edge{ OsmId(6), NodeId(2), NodeId(2) });

  Graph g{ std::move(nodes), std::move(edges) };

  auto offsets = g.getOffsets();

  REQUIRE(offsets[0].out == 0);
  REQUIRE(offsets[1].out == 2);
  REQUIRE(offsets[2].out == 4);
  REQUIRE(offsets[3].out == 4);
  REQUIRE(offsets[4].out == 4);

  REQUIRE(offsets[0].in == 0);
  REQUIRE(offsets[1].in == 1);
  REQUIRE(offsets[2].in == 2);
  REQUIRE(offsets[3].in == 4);
  REQUIRE(offsets[4].in == 4);
}
