#include "catch.hpp"
#include "graph.hpp"

TEST_CASE("Edges remembers source node")
{
  Edge e{ OsmId(1), NodeId(2), NodeId(7) };
  REQUIRE(e.getSourceId().get() == 2);
}
