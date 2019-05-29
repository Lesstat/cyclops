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
#include "edge_load.hpp"

TEST_CASE("Test edge load Computation")
{

  SECTION("No Routes means zero edge load")
  {
    std::vector<Route<2>> v;
    EdgeLoads l(v);
    REQUIRE(l[EdgeId(1)] == 0.0);
    REQUIRE(l[EdgeId(2)] == 0.0);
  }

  SECTION("One Route means load of 1 on contained edges")
  {

    Route<2> r;
    r.edges.push_back(EdgeId(1));

    std::vector<Route<2>> v;
    v.push_back(r);

    EdgeLoads l(v);
    REQUIRE(l[EdgeId(1)] == 1.0);
    REQUIRE(l[EdgeId(2)] == 0.0);
  }

  SECTION("Multiple Routes")
  {

    Route<2> r1;
    r1.edges.push_back(EdgeId(1));
    r1.edges.push_back(EdgeId(2));

    Route<2> r2;
    r2.edges.push_back(EdgeId(2));
    r2.edges.push_back(EdgeId(312));

    std::vector<Route<2>> v;
    v.push_back(r1);
    v.push_back(r2);

    EdgeLoads l(v);
    REQUIRE(l[EdgeId(1)] == 0.5);
    REQUIRE(l[EdgeId(2)] == 1.0);
    REQUIRE(l[EdgeId(312)] == 0.5);
    REQUIRE(l[EdgeId(4)] == 0.0);
  }
}
