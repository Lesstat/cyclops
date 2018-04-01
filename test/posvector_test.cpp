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
#include "posvector.hpp"

TEST_CASE("Add two PosVectors")
{
  PosVector a{ { 1, 0, 0 } };
  PosVector b{ { 1, 2, 3 } };

  PosVector c = a + b;
  REQUIRE(c == PosVector({ 2, 2, 3 }));
}

TEST_CASE("Multiply double to PosVector")
{
  PosVector a{ { 1, 2, 3 } };

  PosVector b = a * 2.0;
  REQUIRE(b == PosVector({ 2, 4, 6 }));
}

TEST_CASE("Differently sized PosVectors throw")
{

  PosVector a{ { 1, 2, 3 } };
  PosVector b{ { 1, 2, 3, 4 } };

  REQUIRE_THROWS(b + a);
}

TEST_CASE("Determine internal angle of three points")
{
  PosVector a{ { 1, 0, 0 } };
  PosVector b{ { 0, 1, 0 } };
  PosVector c{ { 0, 0, 1 } };

  auto angle = b.internalAngle(a, c);
  REQUIRE(angle == Approx(60.0));
  REQUIRE(a.internalAngle(b, c) == c.internalAngle(a, b));
  REQUIRE(angle == c.internalAngle(a, b));
}
