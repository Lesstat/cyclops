#include "catch.hpp"
#include "graph.hpp"

TEST_CASE("Parse Edge from text format")
{

  SECTION("Original Edge")
  {
    Edge e = Edge::createFromText("595292 595293 13 17 30 -1 -1");

    REQUIRE(e.source.get() == 595292);
    REQUIRE(e.destination.get() == 595293);
    REQUIRE(e.cost.length.get() == 13);
    REQUIRE(e.edgeA.has_value() == false);
    REQUIRE(e.edgeB.has_value() == false);
  }

  SECTION("Shortcut Edge")
  {
    Edge e = Edge::createFromText("3 4 20 0 -1 11 2294937");

    REQUIRE(e.source.get() == 3);
    REQUIRE(e.destination.get() == 4);
    REQUIRE(e.cost.length.get() == 20);
    REQUIRE(e.edgeA.has_value());
    REQUIRE(e.edgeA.value() == 11);
    REQUIRE(e.edgeB.has_value());
    REQUIRE(e.edgeB.value() == 2294937);
  }
}
