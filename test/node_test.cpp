#include "catch.hpp"
#include "graph.hpp"

void testNodeInternals(const Node& n, size_t osmId, double lat, double lng, size_t level)
{
  REQUIRE(n.osmId.get() == osmId);
  REQUIRE(n.lat.get() == lat);
  REQUIRE(n.lng.get() == lng);
  REQUIRE(n.level == level);
}

TEST_CASE("Create Node from text repersentation")
{
  Node n = Node::createFromText("0 470552 49.3413737 7.3014905 0 3");
  testNodeInternals(n, 470552, 49.3413737, 7.3014905, 3);
}
