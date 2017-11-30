#include "catch.hpp"
#include "graph.hpp"

TEST_CASE("Create Node from text repersentation")
{
  Node e = Node::createByText("0 470552 49.3413737 7.3014905 0 3");
  REQUIRE(e.osmId.get() == 470552);
  REQUIRE(e.lat.get() == 49.3413737);
  REQUIRE(e.lng.get() == 7.3014905);
  REQUIRE(e.level == 3);
}
