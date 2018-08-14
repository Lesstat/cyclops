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
#include "ilp_independent_set.hpp"
#include <algorithm>

TEST_CASE("Greedy independent set")
{

  std::vector<std::pair<size_t, size_t>> edges;

  REQUIRE(greedy_independent_set(0, edges).empty());

  edges.emplace_back(0, 1);
  edges.emplace_back(1, 2);
  edges.emplace_back(1, 3);
  edges.emplace_back(2, 3);
  edges.emplace_back(3, 4);

  duplicate_edges(edges);

  std::vector<size_t> set = greedy_independent_set(5, edges);
  REQUIRE(set.size() == 3);

  std::sort(set.begin(), set.end());
  REQUIRE(set[0] == 0);
  REQUIRE(set[1] == 2);
  REQUIRE(set[2] == 4);

  set = greedy_independent_set(6, edges);
  REQUIRE(set.size() == 4);

  std::sort(set.begin(), set.end());
  REQUIRE(set[0] == 0);
  REQUIRE(set[1] == 2);
  REQUIRE(set[2] == 4);
  REQUIRE(set[3] == 5);
}
