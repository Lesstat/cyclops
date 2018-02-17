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

std::mutex key;

LinearProgram::LinearProgram(size_t cols)
    : columnCount(cols)
{
  // glp_add_cols(lp, cols);
  lp.setLogLevel(0);

  lp.resize(0, cols);
  for (size_t i = 0; i < cols; ++i) {
    lp.setColLower(i, 0.0);
  }
}

LinearProgram::~LinearProgram() noexcept = default;

void LinearProgram::addConstraint(const std::vector<double>& coeff, double max)
{
  // int row = glp_add_rows(lp, 1);
  // glp_set_row_bnds(lp, row, type, max, max);

  std::vector<int> ind(coeff.size());
  std::vector<double> value(coeff.size());

  int i = 0;
  for (const auto& val : coeff) {
    ind[i] = i;
    value[i] = val;
    ++i;
  }
  lp.addRow(ind.size(), ind.data(), value.data(), -COIN_DBL_MAX, max);

  // glp_set_mat_row(lp, row, coeff.size(), ind.data(), value.data());
}

void LinearProgram::objective(const std::vector<double>& coeff)
{
  // glp_set_obj_dir(lp, GLP_MIN);
  lp.setOptimizationDirection(1);
  for (size_t i = 0; i < coeff.size(); ++i) {
    lp.setObjCoeff(i, coeff[i]);
    // glp_set_obj_coef(lp, i + 1, coeff[i]);
  }
}

bool LinearProgram::solve()
{
  size_t simplex;
  simplex = lp.primal();

  //   size_t status = glp_get_status(lp);
  return lp.isProvenOptimal();
}

double LinearProgram::objectiveFunctionValue()
{
  return lp.objectiveValue();
  //   return glp_get_obj_val(lp);
}

std::vector<double> LinearProgram::variableValues()
{
  auto cols = lp.getColSolution();
  std::vector<double> variables(&cols[0], &cols[columnCount - 1]);

  // for (size_t i = 0; i < columnCount; ++i) {
  //   // variables[i] = glp_get_col_prim(lp, i + 1);
  //   variables[i] = lp.getColSolution();
  // }
  return variables;
}

LinearProgram LinearProgram::setUpLPForContraction()
{
  size_t cols = 3;

  LinearProgram linearProgram;
  linearProgram.columnCount = cols;
  // glp_term_out(GLP_OFF);
  // linearProgram.lp = glp_create_prob();
  // glp_add_cols(linearProgram.lp, cols);
  for (size_t i = 1; i < cols + 1; ++i) {
    // glp_set_col_bnds(linearProgram.lp, i, GLP_LO, 0.0, 0.0);
  }

  // glp_set_obj_dir(linearProgram.lp, GLP_MIN);
  for (size_t i = 0; i < 3; ++i) {
    // glp_set_obj_coef(linearProgram.lp, i + 1, 1.0);
  }

  // int row = glp_add_rows(linearProgram.lp, 1);
  // glp_set_row_bnds(linearProgram.lp, row, GLP_FX, 1.0, 1.0);

  std::vector<int> ind(3 + 1);
  std::vector<double> value(3 + 1);

  int i = 1;
  for (const auto& val : { 1.0, 1.0, 1.0 }) {
    ind[i] = i;
    value[i] = val;
    ++i;
  }

  // glp_set_mat_row(linearProgram.lp, row, 3, ind.data(), value.data());

  return linearProgram;
}

bool LinearProgram::exact() { return exact_; }
void LinearProgram::exact(bool exact) { exact_ = exact; }
