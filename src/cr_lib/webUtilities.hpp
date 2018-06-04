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
#ifndef WEBUTILITIES_H
#define WEBUTILITIES_H

#include "routeComparator.hpp"

std::string routeToJson(const Route& route, const Graph& g, bool writeLogs = false)
{
  std::stringstream resultJson;
  resultJson << "{ \"length\": " << route.costs.length << ", \"height\": " << route.costs.height
             << ", \"unsuitability\": " << route.costs.unsuitability;
  if (writeLogs) {
    auto log = Logger::getInstance();
    resultJson << ", \"debug\":\"" << log->getInfo() << "\" ";
  }
  resultJson << R"(, "route": { "type": "Feature", "geometry": { "type": "LineString", )"
             << "\"coordinates\":[";

  std::unordered_set<NodeId> nodes;
  nodes.reserve(route.edges.size());
  for (const auto& edge : route.edges) {
    nodes.insert(edge.getSourceId());
    nodes.insert(edge.getDestId());
  }

  for (const auto& edge : route.edges) {
    const auto& node = g.getNode(edge.sourcePos());
    resultJson << '[' << node.lng() << ", " << node.lat() << "],";
  }
  if (!route.edges.empty()) {
    const auto& lastEdge = route.edges[route.edges.size() - 1];
    const auto& endNode = g.getNode(lastEdge.destPos());
    resultJson << '[' << endNode.lng() << ", " << endNode.lat() << "]] } } }";
  }
  return resultJson.str();
}
void extractQueryFields(const SimpleWeb::CaseInsensitiveMultimap& queryFields,
    std::optional<size_t>& s, std::optional<size_t>& t, std::optional<size_t>& length,
    std::optional<size_t>& height, std::optional<size_t>& unsuitability)
{
  for (const auto& field : queryFields) {
    if (field.first == "s") {
      s = static_cast<size_t>(stoull(field.second));
    } else if (field.first == "t") {
      t = static_cast<size_t>(stoull(field.second));
    } else if (field.first == "length") {
      length = static_cast<size_t>(stoull(field.second));
    } else if (field.first == "height") {
      height = static_cast<size_t>(stoull(field.second));
    } else if (field.first == "unsuitability") {
      unsuitability = static_cast<size_t>(stoull(field.second));
    }
  }
}

void appendAlternativesToJsonStream(
    std::stringstream& result, const AlternativeRoutes& routes, const Graph& g)
{

  result << R"({ "config1": ")" << std::round(routes.config1.length * 100) << "/"
         << std::round(routes.config1.height * 100) << "/"
         << std::round(routes.config1.unsuitability * 100) << R"(", )";
  result << R"( "route1":  )" << routeToJson(routes.route1, g) << ", ";

  result << R"( "config2": ")" << std::round(routes.config2.length * 100) << "/"
         << std::round(routes.config2.height * 100) << "/"
         << std::round(routes.config2.unsuitability * 100) << R"(", )";
  result << R"( "route2":  )" << routeToJson(routes.route2, g) << ", ";
  result << R"( "shared":  )" << std::round(routes.shared * 100) << ", "
         << R"( "frechet":  )" << routes.frechet << "}";
}

void appendCsvLine(std::stringstream& result, const std::string& method, NodePos from, NodePos to,
    size_t threshold, size_t maxSplits, size_t maxLevel, size_t maxRepeating,
    const std::vector<TriangulationPoint>& routes, size_t routeCount, size_t time)
{

  size_t lastInterestingRoute = 0;
  size_t setSize = 0;
  size_t nonIdenticalRoutes = 0;
  double lowestSharing = 1;
  for (size_t i = 0; i < routes.size(); ++i) {
    const auto& route = routes[i];
    if (route.selected) {
      setSize++;
      lastInterestingRoute = i;
    }
    nonIdenticalRoutes++;
    for (size_t j = i + 1; j < routes.size(); ++j) {
      double sharing = calculateSharing(route.route, routes[j].route);
      if (sharing < lowestSharing) {
        lowestSharing = sharing;
      }
      if (sharing > 0.99) {
        nonIdenticalRoutes--;
        break;
      }
    }
  }

  result << from << "," << to << "," << method << "," << threshold << "," << maxSplits << ","
         << maxLevel << "," << maxRepeating << "," << setSize << "," << nonIdenticalRoutes << ","
         << routeCount << "," << lastInterestingRoute << "," << lowestSharing << "," << time
         << '\n';
}

#endif /* WEBUTILITIES_H */
