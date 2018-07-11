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
#ifndef CONTRACTIONLP_H
#define CONTRACTIONLP_H

#include <boost/process.hpp>

namespace bp = boost::process;
class ContractionLp {
  public:
  ContractionLp()
      : lp("./build/cr_lp", std::to_string(Cost::dim), bp::std_out > lpOutput,
            bp::std_in < lpInput){};
  ContractionLp(const ContractionLp& other) = delete;
  ContractionLp(ContractionLp&& other) = default;
  virtual ~ContractionLp() noexcept = default;
  ContractionLp& operator=(const ContractionLp& other) = delete;
  ContractionLp& operator=(ContractionLp&& other) = default;

  void addConstraint(const double (&coeff)[Cost::dim])
  {
    for (size_t i = 0; i < Cost::dim; ++i) {
      lpInput << coeff[i] << ' ';
    }
    lpInput << '\n';
  }
  bool solve()
  {
    lpInput << '\n';
    lpInput.flush();
    lpOutput >> lpResult;
    if (lpResult == "Infeasible") {
      return false;
    }
    return true;
  }
  std::vector<double> variableValues()
  {

    std::vector<double> result;
    result.push_back(std::stod(lpResult));
    double coeff = 0;
    for (size_t i = 1; i < Cost::dim; ++i) {
      lpOutput >> coeff;
      result.push_back(coeff);
    }
    return result;
  }

  protected:
  private:
  bp::ipstream lpOutput;
  bp::opstream lpInput;
  bp::child lp;
  std::string lpResult;
};

#endif /* CONTRACTIONLP_H */
