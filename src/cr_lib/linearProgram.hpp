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

  size_t constraintCount() const;

  static LinearProgram setUpLPForContraction();

  protected:
  private:
  LinearProgram() = default;
  std::optional<ClpSimplex> lp;
  size_t columnCount;

  struct Constraint {
    std::vector<double> coeff;
    double max;
    double min;
    Constraint(std::vector<double> coeff, double max, double min)
        : coeff(coeff)
        , max(max)
        , min(min)
    {
    }
  };
  std::vector<Constraint> constraints;
  std::optional<std::vector<double>> objective_;
};

#endif /* LINEARPROGRAM_H */
