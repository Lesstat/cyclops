/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2019  Florian Barth

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
#include "restricted_set.hpp"

TEST_CASE("Combining Empty exclusion_sets yields empty exclusion_set")
{
  auto result = combine({}, {});
  REQUIRE(result.empty());
}

TEST_CASE("Exclusion_sets must be same size for combining")
{
  REQUIRE_THROWS(combine({}, { true }));
}

TEST_CASE("Any excluded node must be avoided in combined exclusion_set")
{

  auto result = combine({ false, true, true, false }, { false, false, true, true });
  exclusion_set expected = { false, true, true, true };
  REQUIRE(result.size() == expected.size());
  for (size_t i = 0; i < result.size(); ++i) {
    REQUIRE(result[i] == expected[i]);
  }
}
