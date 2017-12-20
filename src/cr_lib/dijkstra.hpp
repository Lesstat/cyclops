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
#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "graph.hpp"
#include <deque>
#include <vector>

using LengthConfig = NamedType<double, struct LengthConfigParameter>;
using HeightConfig = NamedType<double, struct HeightConfigParameter>;
using UnsuitabilityConfig = NamedType<double, struct UnsuitabilityConfigParameter>;

struct Config {
  LengthConfig length;
  HeightConfig height;
  UnsuitabilityConfig unsuitability;

  Config(LengthConfig l, HeightConfig h, UnsuitabilityConfig u)
      : length(l)
      , height(h)
      , unsuitability(u)
  {
  }
};

struct Route {
  Cost costs;
  std::deque<Edge> edges;
};

class Dijkstra {
  public:
  Dijkstra(Graph* g, size_t nodeCount);
  Dijkstra(const Dijkstra& other) = default;
  Dijkstra(Dijkstra&& other) noexcept;
  virtual ~Dijkstra() noexcept = default;
  Dijkstra& operator=(const Dijkstra& other) = default;
  Dijkstra& operator=(Dijkstra&& other) noexcept = default;

  std::optional<Route> findBestRoute(NodePos from, NodePos to, Config config);

  private:
  void clearState();

  using NodeToEdgeMap = std::unordered_map<NodePos, EdgeId>;
  Route buildRoute(NodePos node, NodeToEdgeMap previousEdgeS, NodeToEdgeMap previousEdgeT, NodePos from, NodePos to);
  std::vector<double> costS;
  std::vector<double> costT;
  std::vector<NodePos> touchedS;
  std::vector<NodePos> touchedT;
  Graph* graph;
};

#endif /* DIJKSTRA_H */
