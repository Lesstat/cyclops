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

#ifndef POSVECTOR_H
#define POSVECTOR_H
#include "dijkstra.hpp"

class PosVector {
  public:
  PosVector(const std::vector<double>&& values);
  PosVector(const PosVector& other) = default;
  PosVector(PosVector&& other) noexcept = default;
  virtual ~PosVector() noexcept = default;
  PosVector& operator=(const PosVector& other) = default;
  PosVector& operator=(PosVector&& other) noexcept = default;

  PosVector& operator+=(const PosVector& other);
  PosVector operator+(const PosVector& other) const;
  bool operator==(const PosVector& other) const;
  PosVector& operator*=(double n);
  PosVector operator*(double n) const;

  operator Config() const;

  protected:
  private:
  void checkSizes(const size_t& otherSize) const;
  std::vector<double> values;
};

#endif /* POSVECTOR_H */
