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
#include "restricted_set.hpp"

exclusion_set combine(const exclusion_set& a, const exclusion_set& b)
{
  if (a.size() != b.size()) {
    throw std::runtime_error("Trying to combine sets of different sizes");
  }
  exclusion_set result(a.size(), false);
  for (size_t i = 0; i < a.size(); ++i) {
    result[i] = a[i] || b[i];
  }
  return result;
}
