#include "catch.hpp"
#include "graph.hpp"

void testEdgeInternals(const Edge& e,
    size_t source,
    size_t destination,
    double length,
    double height,
    double unsuitability,
    const ReplacedEdge& edgeA,
    const ReplacedEdge& edgeB)
{
  REQUIRE(e.source == source);
  REQUIRE(e.destination == destination);
  REQUIRE(e.cost.length == length);
  REQUIRE(e.cost.height == height);
  REQUIRE(e.cost.unsuitability == unsuitability);
  REQUIRE(e.edgeA == edgeA);
  REQUIRE(e.edgeB == edgeB);
}

TEST_CASE("Parse Edge from text format")
{

  SECTION("Original Edge")
  {
    Edge e = Edge::createFromText("595292 595293 13 17 30 -1 -1");
    testEdgeInternals(e, 595292, 595293, 13, 0, 0, {}, {});
  }

  SECTION("Shortcut Edge")
  {
    Edge e = Edge::createFromText("3 4 20 0 -1 11 2294937");
    testEdgeInternals(e, 3, 4, 20, 0, 0, 11, 2294937);
  }
}
