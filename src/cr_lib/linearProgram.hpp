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
#ifndef LINEARPROGRAM_H
#define LINEARPROGRAM_H

#include "ClpSimplex.hpp"
#include <vector>

class LinearProgram {
  public:
  LinearProgram(size_t cols);
  LinearProgram(const LinearProgram& other) = default;
  LinearProgram(LinearProgram&& other) = default;
  virtual ~LinearProgram() noexcept;
  LinearProgram& operator=(const LinearProgram& other) = default;
  LinearProgram& operator=(LinearProgram&& other) = default;

  void addConstraint(const std::vector<double>& coeff, double max, double min = -COIN_DBL_MAX);
  void objective(const std::vector<double>& coeff);
  bool solve();
  double objectiveFunctionValue();
  std::vector<double> variableValues();

  static LinearProgram setUpLPForContraction();

  bool exact();
  void exact(bool exact);

  protected:
  private:
  LinearProgram() = default;
  ClpSimplex lp;
  size_t columnCount;
  size_t rowCount = 0;
  bool exact_ = false;
};

#endif /* LINEARPROGRAM_H */
