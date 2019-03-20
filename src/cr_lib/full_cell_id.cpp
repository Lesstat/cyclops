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

#include "enumerate_optimals.hpp"

FullCellId::FullCellId() { data = std::make_shared<std::pair<bool, double>>(false, -1.0); }

FullCellId::FullCellId(const FullCellId& other) = default;
FullCellId& FullCellId::operator=(const FullCellId& rhs) = default;
FullCellId& FullCellId::operator=(FullCellId&& rhs) = default;
FullCellId::FullCellId(FullCellId&& other) = default;
FullCellId::~FullCellId() = default;

bool FullCellId::alive() const { return data.use_count() > 1; }
bool FullCellId::checked() const { return std::get<bool>(*data); }
void FullCellId::checked(bool check) { std::get<bool>(*data) = check; }
double FullCellId::prio() const { return std::get<double>(*data); }
void FullCellId::prio(double p) const { std::get<double>(*data) = p; }

bool FullCellId::operator==(const FullCellId& other) const { return data == other.data; }
