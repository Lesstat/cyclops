#include "catch.hpp"
#include "graph.hpp"

void testNodeInternals(const Node& n, OsmId osmId, Lat lat, Lng lng, Height height, size_t level)
{
  REQUIRE(n.osmId == osmId);
  REQUIRE(n.lat == lat);
  REQUIRE(n.lng == lng);
  REQUIRE(n.height == height);
  REQUIRE(n.level == level);
}

TEST_CASE("Create Node from text repersentation")
{

  Node n = Node::createFromText("0 163361 48.6478807 9.3334938 300 2");
  testNodeInternals(n, OsmId{ 163361 }, Lat{ 48.6478807 }, Lng{ 9.3334938 }, Height{ 300 }, 2);
}
