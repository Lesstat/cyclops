/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2019  Florian Barth

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

#include "url_parsing.hpp"
#include "enumerate_optimals.hpp"

#include <sstream>

ImportantMetric parse_important_metric(const std::string& value)
{
  if (value == "")
    throw std::runtime_error(
        "Illegal format for important metric and slack. Expected \"Index,slack\"");

  std::istringstream ss(value);
  std::string line;
  std::getline(ss, line, ',');
  size_t metric = std::stoi(line);

  std::getline(ss, line);
  double slack = std::stod(line);

  return ImportantMetric { metric, slack };
}

std::vector<ImportantMetric> parse_important_metric_list(const std::string& metrics)
{
  std::vector<ImportantMetric> result;
  std::istringstream ss(metrics);
  for (std::string metric; std::getline(ss, metric, ';');) {
    result.push_back(parse_important_metric(metric));
  }
  return result;
}
