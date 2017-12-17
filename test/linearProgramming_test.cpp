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
#include "linearProgram.hpp"

TEST_CASE("Solving a simple LP")
{
  LinearProgram lp{ 4 };
  lp.objective({ 7.0, 3.0, 2.0, 4.0 });
  lp.addConstraint({ -1.0, -2.0, -4.0, -1.0 }, -11.0);
  lp.addConstraint({ -3.0, -2.0, -1.0, -4.0 }, -7.0);
  lp.addConstraint({ -5.0, -0.0, -0.0, -2.0 }, -5.0);
  lp.addConstraint({ -7.0, -3.0, -2.0, -4.0 }, 0.0);

  lp.solve();
  double val = lp.objectiveFunctionValue();
  REQUIRE(val == Approx(12.5094));
}
