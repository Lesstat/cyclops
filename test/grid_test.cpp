/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2018  Florian Barth

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
#include "grid.hpp"

TEST_CASE("Find next Node, with only one node")
{
  std::vector<Node> nodes{};
  nodes.emplace_back(NodeId{ 0 }, Lat{ 48.5 }, Lng{ 8.3 });

  Grid grid{ nodes };
  REQUIRE(grid.findNextNode(Lat{ 47.123 }, Lng{ 9.2 }) == NodePos{ 0 });
}

TEST_CASE("Find next Node, with two nodes")
{
  std::vector<Node> nodes{};
  nodes.emplace_back(NodeId{ 0 }, Lat{ 48.5 }, Lng{ 8.3 });
  nodes.emplace_back(NodeId{ 1 }, Lat{ 47.123 }, Lng{ 9.2 });

  Grid grid{ nodes };
  REQUIRE(grid.findNextNode(Lat{ 47.123 }, Lng{ 9.2 }) == NodePos{ 1 });
}

TEST_CASE("Find next Node, with great distance")
{
  std::vector<Node> nodes{};
  nodes.emplace_back(NodeId{ 0 }, Lat{ 45 }, Lng{ 9 });
  nodes.emplace_back(NodeId{ 1 }, Lat{ 52 }, Lng{ 14.5 });
  nodes.emplace_back(NodeId{ 2 }, Lat{ 60 }, Lng{ 20 });

  Grid grid{ nodes };
  REQUIRE(grid.findNextNode(Lat{ 55 }, Lng{ 16 }).value() == NodePos{ 1 });
}
