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

#include "server_http.hpp"
#include "json/json.h"

template <int Dim>
Json::Value routeToJson(const Route<Dim>& route, const Graph<Dim>& g, bool writeLogs = false)
{
  using Edge = Edge<Dim>;

  Json::Value result;
  Json::Value costs(Json::arrayValue);

  for (const auto& v : route.costs.values) {
    costs.append(v);
  }
  result["costs"] = costs;

  if (writeLogs) {
    auto log = Logger::getInstance();
    result["debug"] = log->getInfo();
  }
  Json::Value js_route;
  js_route["type"] = "Feature";

  Json::Value geometry;
  geometry["type"] = "LineString";

  std::unordered_set<NodeId> nodes;
  nodes.reserve(route.edges.size());
  for (const auto& edge : route.edges) {
    nodes.insert(Edge::getSourceId(edge));
    nodes.insert(Edge::getDestId(edge));
  }

  Json::Value coordinates(Json::arrayValue);

  for (const auto& edge : route.edges) {
    const auto& node = g.getNode(Edge::sourcePos(edge));
    Json::Value js_node(Json::arrayValue);
    js_node.append(node.lng().get());
    js_node.append(node.lat().get());
    js_node.append(node.height());
    coordinates.append(js_node);
  }
  if (!route.edges.empty()) {
    const auto& lastEdge = route.edges[route.edges.size() - 1];
    const auto& endNode = g.getNode(Edge::destPos(lastEdge));

    Json::Value js_node(Json::arrayValue);
    js_node.append(endNode.lng().get());
    js_node.append(endNode.lat().get());
    js_node.append(endNode.height());
    coordinates.append(js_node);
  }

  geometry["coordinates"] = coordinates;
  js_route["geometry"] = geometry;

  result["route"] = js_route;
  return result;
}

void extractQueryFields(const SimpleWeb::CaseInsensitiveMultimap& queryFields,
    std::optional<uint32_t>& s, std::optional<uint32_t>& t, std::optional<uint32_t>& length,
    std::optional<uint32_t>& height, std::optional<uint32_t>& unsuitability)
{
  for (const auto& field : queryFields) {
    if (field.first == "s") {
      s = static_cast<uint32_t>(stoul(field.second));
    } else if (field.first == "t") {
      t = static_cast<uint32_t>(stoul(field.second));
    } else if (field.first == "length") {
      length = static_cast<uint32_t>(stoul(field.second));
    } else if (field.first == "height") {
      height = static_cast<uint32_t>(stoul(field.second));
    } else if (field.first == "unsuitability") {
      unsuitability = static_cast<uint32_t>(stoull(field.second));
    }
  }
}

#endif /* WEBUTILITIES_H */
