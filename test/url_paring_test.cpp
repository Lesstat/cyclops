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

#include "catch.hpp"
#include "restriction_policy.hpp"
#include "url_parsing.hpp"

TEST_CASE("Empty metric Query String throws") { REQUIRE_THROWS(parse_important_metric("")); }
TEST_CASE("Textual metric Query String throws")
{
  REQUIRE_THROWS(parse_important_metric("distance,1.4"));
}

TEST_CASE("Important metric should consist of index and slack comma delimited")
{
  {
    auto result = parse_important_metric("1,1.5");

    REQUIRE(result.metric == 1);
    REQUIRE(result.slack == 1.5);
  }

  {
    auto result = parse_important_metric("2,1.1");

    REQUIRE(result.metric == 2);
    REQUIRE(result.slack == 1.1);
  }
}

TEST_CASE("Empty Query String for metrics gives empty vector")
{
  REQUIRE(parse_important_metric_list("").empty());
}

TEST_CASE("Metric list parser works for single item without delimiter")
{
  auto metrics = parse_important_metric_list("2,1.7");
  REQUIRE(metrics.size() == 1);

  auto& metric = metrics.front();
  REQUIRE(metric.metric == 2);
  REQUIRE(metric.slack == 1.7);
}

TEST_CASE("Metric list parser can parse two metrics at once")
{
  auto metrics = parse_important_metric_list("2,1.7;3,1.2;");
  REQUIRE(metrics.size() == 2);

  {
    auto& metric = metrics.front();
    REQUIRE(metric.metric == 2);
    REQUIRE(metric.slack == 1.7);
  }

  {
    auto& metric = metrics.back();
    REQUIRE(metric.metric == 3);
    REQUIRE(metric.slack == 1.2);
  }
}

TEST_CASE("Convert Empty Important Metrics Vector to only false arrays for Enumerate Optimals")
{
  std::vector<ImportantMetric> m;
  auto slacks = important_metrics_to_array<3>(m);

  for (auto& slack : slacks)
    REQUIRE(slack == std::numeric_limits<double>::max());
}

TEST_CASE("Convert place Important Metric and slack at right position")
{
  std::vector<ImportantMetric> m;
  m.push_back(ImportantMetric { 1, 1.3 });

  auto slacks = important_metrics_to_array<3>(m);

  for (size_t i = 0; i < 3; ++i)
    if (i == 1) {
      REQUIRE(slacks[i] == 1.3);
    } else {
      REQUIRE(slacks[i] == std::numeric_limits<double>::max());
    }
}
