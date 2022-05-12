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
#include <sstream>

void testNodeInternals(const Node& n, NodeId id, Lat lat, Lng lng, size_t level)
{
  REQUIRE(n.id_ == id);
  REQUIRE(n.lat_ == lat);
  REQUIRE(n.lng_ == lng);
  REQUIRE(n.level == level);
}

TEST_CASE("Create Node from text repersentation")
{
  std::stringstream ss("0 163361 48.6478807 9.3334938 300 2");
  Node n = Node::createFromText(ss);
  testNodeInternals(n, NodeId { 0 }, Lat { 48.6478807 }, Lng { 9.3334938 }, 2);
}
