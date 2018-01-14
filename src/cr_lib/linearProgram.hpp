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

#include "glpk.h"
#include <vector>

class LinearProgram {
  public:
  LinearProgram(size_t cols);
  LinearProgram(const LinearProgram& other) = default;
  LinearProgram(LinearProgram&& other) noexcept = default;
  virtual ~LinearProgram() noexcept;
  LinearProgram& operator=(const LinearProgram& other) = default;
  LinearProgram& operator=(LinearProgram&& other) noexcept = default;

  void addConstraint(const std::vector<double>& coeff, double max, size_t type = GLP_UP);
  void objective(const std::vector<double>& coeff);
  bool solve();
  double objectiveFunctionValue();
  std::vector<double> variableValues();
  void exact(bool exact);
  bool exact();

  protected:
  private:
  glp_prob* lp;
  size_t columnCount;
  bool exact_ = false;
};

#endif /* LINEARPROGRAM_H */
