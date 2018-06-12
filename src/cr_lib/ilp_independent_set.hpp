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

#ifndef ILP_INDEPENDENT_SET_H
#define ILP_INDEPENDENT_SET_H

#include "glpk.h"

class ilpSet {

  glp_prob* problem;
  size_t nodeCount;

  public:
  ilpSet(size_t nodeCount)
      : nodeCount(nodeCount)
  {
    glp_term_out(GLP_OFF);
    problem = glp_create_prob();
    glp_add_cols(problem, nodeCount);
    glp_set_obj_dir(problem, GLP_MAX);
    for (size_t i = 1; i <= nodeCount; ++i) {
      glp_set_obj_coef(problem, i, 1);
      glp_set_col_kind(problem, i, GLP_BV);
    }
  }
  ~ilpSet() { glp_delete_prob(problem); }

  void addEdges(std::vector<std::pair<size_t, size_t>>& edges)
  {
    glp_add_rows(problem, edges.size());
    for (size_t i = 1; i <= edges.size(); ++i) {
      auto [u, v] = edges[i - 1];
      std::vector<int> indices = { 0, static_cast<int>(u) + 1, static_cast<int>(v) + 1 };
      std::vector<double> values = { 0, 1, 1 };
      glp_set_mat_row(problem, i, 2, indices.data(), values.data());
      glp_set_row_bnds(problem, i, GLP_UP, 0, 1);
    }
  }

  std::vector<size_t> find_set()
  {
    auto status = glp_simplex(problem, nullptr);
    if (status != 0) {
      throw std::invalid_argument(
          "GLPK returned " + std::to_string(status) + " for LP relaxiation");
    }
    glp_iocp params;
    glp_init_iocp(&params);

    params.tm_lim = 5000;

    status = glp_intopt(problem, &params);
    if (status != 0) {
      throw std::invalid_argument("GLPK returned " + std::to_string(status) + " for ILP");
    }
    std::vector<size_t> results;

    status = glp_mip_status(problem);
    switch (status) {
    case GLP_OPT:
    case GLP_FEAS: {
      for (size_t i = 1; i <= nodeCount; ++i) {
        if (glp_mip_col_val(problem, i) >= 0.9) {
          results.push_back(i - 1);
        }
      }
      break;
    }
    default:
      throw std::invalid_argument("Not feasible/ optimal: " + std::to_string(status));
      break;
    }

    return results;
  }
};

std::vector<size_t> find_independent_set(
    size_t nodeCount, std::vector<std::pair<size_t, size_t>>& edges);

#endif /* ILP_INDEPENDENT_SET_H */
