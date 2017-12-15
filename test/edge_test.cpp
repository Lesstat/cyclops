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
#include "graph.hpp"

void testEdgeInternals(const Edge& e,
    NodeId source,
    NodeId destination,
    Length length,
    Height height,
    Unsuitability unsuitability,
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
    Edge e = Edge::createFromText("10990 689504 24.340902087980123 7 4 -1 -1");
    testEdgeInternals(e, NodeId{ 10990 }, NodeId{ 689504 }, Length{ 24.340902087980123 }, Height{ 7 }, Unsuitability{ 4 }, {}, {});
  }

  SECTION("Shortcut Edge")
  {
    Edge e = Edge::createFromText("10990 689504 24.340902087980123 7 4 259 687");
    testEdgeInternals(e, NodeId{ 10990 }, NodeId{ 689504 }, Length{ 24.340902087980123 }, Height{ 7 }, Unsuitability{ 4 }, EdgeId{ 259 }, EdgeId{ 687 });
  }
}
