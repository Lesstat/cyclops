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

template <int D>
void testEdgeInternals(const Edge<D>& e, NodeId source, NodeId destination, Length length,
    Height height, Unsuitability unsuitability, const ReplacedEdge& edgeA,
    const ReplacedEdge& edgeB)
{
  REQUIRE(e.source == source);
  REQUIRE(e.destination == destination);
  REQUIRE(D == 3);
  REQUIRE(e.cost.values[0] == length);
  REQUIRE(e.cost.values[1] == height);
  REQUIRE(e.cost.values[2] == unsuitability);
  REQUIRE(e.edgeA == edgeA);
  REQUIRE(e.edgeB == edgeB);
}
TEST_CASE("Parse Edge from text format")
{

  SECTION("Original Edge")
  {
    std::stringstream ss("10990 689504 24.340902087980123 7 12.1758 -1 -1");
    Edge e = Edge<3>::createFromText(ss);
    testEdgeInternals(e, NodeId { 10990 }, NodeId { 689504 }, Length { 24.340902087980123 },
        Height { 7 }, Unsuitability { 12.1758 }, {}, {});
  }

  SECTION("Shortcut Edge")
  {
    std::stringstream ss("10990 689504 24.340902087980123 7 12.1758 259 687");
    Edge e = Edge<3>::createFromText(ss);
    testEdgeInternals(e, NodeId { 10990 }, NodeId { 689504 }, Length { 24.340902087980123 },
        Height { 7 }, Unsuitability { 12.1758 }, EdgeId { 259 }, EdgeId { 687 });
  }
}
