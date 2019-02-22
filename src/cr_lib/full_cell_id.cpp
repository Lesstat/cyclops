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

int FullCellId::currentId = 0;
std::vector<int> FullCellId::alive_ = std::vector<int>();
std::vector<bool> FullCellId::checked_ = std::vector<bool>();
std::vector<double> FullCellId::prio_ = std::vector<double>();

FullCellId::FullCellId()
    : id_(currentId++)
{
  alive_.push_back(1);
  checked_.push_back(false);
  prio_.push_back(-1.0);
}

FullCellId::FullCellId(const FullCellId& other)
    : id_(other.id_)
{
  alive_[id_]++;
}

FullCellId& FullCellId::operator=(const FullCellId& rhs)
{
  // Check for self-assignment!
  if (this == &rhs)
    return *this;

  alive_[id_]--;
  id_ = rhs.id_;
  alive_[id_]++;
  return *this;
}

FullCellId& FullCellId::operator=(const FullCellId&& rhs)
{
  *this = rhs;
  return *this;
}

FullCellId::FullCellId(FullCellId&& other)
{
  id_ = other.id_;
  alive_[id_]++;
}

FullCellId::~FullCellId() { alive_[id_]--; }

bool FullCellId::alive() const { return alive_[id_] > 1; }
bool FullCellId::checked() const { return checked_[id_]; }
void FullCellId::checked(bool check) { checked_[id_] = check; }
size_t FullCellId::id() const { return id_; }
double FullCellId::prio() const { return prio_[id_]; }
void FullCellId::prio(double p) const { prio_[id_] = p; }

bool FullCellId::operator==(const FullCellId& other) const { return id_ == other.id_; }
