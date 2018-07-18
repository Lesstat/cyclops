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

#include "posvector.hpp"
#include <cmath>

PosVector::PosVector(std::vector<double>&& values)
    : values(std::move(values))
{
}

PosVector& PosVector::operator+=(const PosVector& other)
{
  checkSizes(other.values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    values[i] += other.values[i];
  }
  return *this;
}
PosVector PosVector::operator+(const PosVector& other) const
{
  auto result = *this;
  return result += other;
}
bool PosVector::operator==(const PosVector& other) const
{
  return std::equal(values.begin(), values.end(), other.values.begin(), other.values.end());
}

PosVector& PosVector::operator*=(double n)
{
  for (auto& value : values) {
    value *= n;
  }
  return *this;
}
PosVector PosVector::operator*(double n) const
{
  auto result = *this;
  return result *= n;
}

void PosVector::checkSizes(const size_t& otherSize) const
{
  if (values.size() != otherSize) {
    throw std::invalid_argument("PosVectors have different sizes (" + std::to_string(values.size())
        + " & " + std::to_string(otherSize) + ")");
  }
}

PosVector::operator Config() const
{
  checkSizes(Cost::dim);
  return Config(values);
}

double PosVector::distance(const PosVector& other) const
{
  checkSizes(other.values.size());

  double sum = 0;
  for (size_t i = 0; i < values.size(); ++i) {
    sum += std::pow(values[i] - other.values[i], 2);
  }
  return std::sqrt(sum);
}

double PosVector::internalAngle(const PosVector& first, const PosVector& second) const
{
  PosVector firstSlope = *this + (first * -1);
  PosVector secondSlope = *this + (second * -1);
  auto res = std::acos((firstSlope * secondSlope) / (firstSlope.length() * secondSlope.length()));
  return res * 180.0 / M_PI;
}

double PosVector::length() const
{
  double sum = 0.0;

  for (const auto& value : values) {
    sum += value * value;
  }
  return std::sqrt(sum);
}

double PosVector::operator*(PosVector other) const
{
  checkSizes(other.values.size());
  double result = 0.0;

  for (size_t i = 0; i < values.size(); ++i) {
    result += values[i] * other.values[i];
  }
  return result;
}
