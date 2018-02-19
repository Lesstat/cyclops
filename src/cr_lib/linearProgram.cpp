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
#include "linearProgram.hpp"
#include <mutex>

LinearProgram::LinearProgram(size_t cols)
    : columnCount(cols)
{
  lp.setLogLevel(0);

  lp.resize(0, cols);
  for (size_t i = 0; i < cols; ++i) {
    lp.setColLower(i, 0.0);
  }
}

LinearProgram::~LinearProgram() noexcept = default;

void LinearProgram::addConstraint(const std::vector<double>& coeff, double max, double min)
{

  std::vector<int> ind(coeff.size());
  std::vector<double> value(coeff.size());

  int i = 0;
  for (const auto& val : coeff) {
    ind[i] = i;
    value[i] = val;
    ++i;
  }
  lp.addRow(ind.size(), ind.data(), value.data(), min, max);
}

void LinearProgram::objective(const std::vector<double>& coeff)
{
  lp.setOptimizationDirection(1);
  for (size_t i = 0; i < coeff.size(); ++i) {
    lp.setObjCoeff(i, coeff[i]);
  }
}

bool LinearProgram::solve()
{
  lp.primal();
  return !lp.isProvenPrimalInfeasible();
}

double LinearProgram::objectiveFunctionValue() { return lp.objectiveValue(); }

std::vector<double> LinearProgram::variableValues()
{
  auto cols = lp.getColSolution();
  std::vector<double> variables(&cols[0], &cols[columnCount]);

  return variables;
}

bool LinearProgram::exact() { return exact_; }
void LinearProgram::exact(bool exact) { exact_ = exact; }
