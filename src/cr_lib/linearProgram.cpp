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
#include <mutex>

std::mutex key;

LinearProgram::LinearProgram(size_t cols)
    : columnCount(cols)
{
  std::lock_guard guard{ key };
  glp_term_out(GLP_OFF);
  lp = glp_create_prob();
  glp_add_cols(lp, cols);
  for (size_t i = 1; i < cols + 1; ++i) {
    glp_set_col_bnds(lp, i, GLP_LO, 0.0, 0.0);
  }
}

LinearProgram::~LinearProgram() noexcept
{
  std::lock_guard guard{ key };
  glp_delete_prob(lp);
}

void LinearProgram::addConstraint(const std::vector<double>& coeff, double max, size_t type)
{
  std::lock_guard guard{ key };
  int row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, type, max, max);

  std::vector<int> ind(coeff.size() + 1);
  std::vector<double> value(coeff.size() + 1);

  int i = 1;
  for (const auto& val : coeff) {
    ind[i] = i;
    value[i] = val;
    ++i;
  }

  glp_set_mat_row(lp, row, coeff.size(), ind.data(), value.data());
}

void LinearProgram::objective(const std::vector<double>& coeff)
{
  std::lock_guard guard{ key };
  glp_set_obj_dir(lp, GLP_MIN);
  for (size_t i = 0; i < coeff.size(); ++i) {
    glp_set_obj_coef(lp, i + 1, coeff[i]);
  }
}

bool LinearProgram::solve()
{
  std::lock_guard guard{ key };
  size_t simplex;
  if (!exact_) {
    simplex = glp_simplex(lp, nullptr);
  } else {
    simplex = glp_exact(lp, nullptr);
  }
  size_t status = glp_get_status(lp);
  return simplex == 0 && status == GLP_OPT;
}

double LinearProgram::objectiveFunctionValue()
{
  std::lock_guard guard{ key };
  return glp_get_obj_val(lp);
}

std::vector<double> LinearProgram::variableValues()
{
  std::lock_guard guard{ key };
  std::vector<double> variables(columnCount);
  for (size_t i = 0; i < columnCount; ++i) {
    variables[i] = glp_get_col_prim(lp, i + 1);
  }
  return variables;
}

void LinearProgram::exact(bool exact) { exact_ = exact; }

bool LinearProgram::exact() { return exact_; }
