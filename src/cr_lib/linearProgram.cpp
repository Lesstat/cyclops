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
#include "glpk.h"

LinearProgram::LinearProgram(size_t cols)
{
  lp = glp_create_prob();
  glp_add_cols(lp, cols);
  for (size_t i = 1; i < cols + 1; ++i) {
    glp_set_col_bnds(lp, i, GLP_LO, 0.0, 0.0);
  }
}

LinearProgram::~LinearProgram() noexcept
{
  glp_delete_prob(lp);
}
void LinearProgram::addConstraint(const std::vector<double>& coeff, double max)
{
  int row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_UP, 0.0, max);

  auto* ind = new int[coeff.size() + 1];
  auto* value = new double[coeff.size() + 1];

  int i = 1;
  for (const auto& val : coeff) {
    ind[i] = i;
    value[i] = val;
    ++i;
  }

  glp_set_mat_row(lp, row, coeff.size(), ind, value);
  delete[] ind;
  delete[] value;
}

void LinearProgram::objective(const std::vector<double>& coeff)
{
  glp_set_obj_dir(lp, GLP_MIN);
  for (size_t i = 0; i < coeff.size(); ++i) {
    glp_set_obj_coef(lp, i + 1, coeff[i]);
  }
}

void LinearProgram::solve()
{
  glp_simplex(lp, nullptr);
}

double LinearProgram::objectiveFunctionValue()
{
  return glp_get_obj_val(lp);
}
