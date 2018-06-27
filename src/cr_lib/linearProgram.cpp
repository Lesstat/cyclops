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
}

LinearProgram::~LinearProgram() noexcept = default;

void LinearProgram::addConstraint(const std::vector<double>& coeff, double max, double min)
{
  constraints.emplace_back(coeff, max, min);
}

void LinearProgram::objective(const std::vector<double>& coeff) { objective_ = coeff; }

bool LinearProgram::solve()
{
  if (!lp) {
    lp = ClpSimplex();
    lp->setLogLevel(0);

    lp->resize(0, columnCount);
    for (size_t i = 0; i < columnCount; ++i) {
      lp->setColLower(i, 0.0);
    }
  }
  if (objective_) {
    lp->setOptimizationDirection(1);
    for (size_t i = 0; i < objective_->size(); ++i) {
      lp->setObjCoeff(i, (*objective_)[i]);
    }
    objective_ = {};
  }
  if (!constraints.empty()) {

    for (auto& c : constraints) {
      std::vector<int> ind(c.coeff.size());
      std::vector<double> value(c.coeff.size());

      int i = 0;
      for (const auto& val : c.coeff) {
        ind[i] = i;
        value[i] = val;
        ++i;
      }
      lp->addRow(ind.size(), ind.data(), value.data(), c.min, c.max);
    }
    constraints.clear();
  }
  lp->primal();
  return lp->primalFeasible();
}

double LinearProgram::objectiveFunctionValue() { return lp->objectiveValue(); }

std::vector<double> LinearProgram::variableValues()
{
  auto cols = lp->getColSolution();
  std::vector<double> variables(&cols[0], &cols[columnCount]);
  return variables;
}

size_t LinearProgram::constraintCount() const { return lp ? lp->getNumRows() : 0; }
