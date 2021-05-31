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
#include <cmath>
#include <iostream>
#include <queue>
#include <random>

using LengthConfig = NamedType<double, struct LengthConfigParameter>;
using HeightConfig = NamedType<double, struct HeightConfigParameter>;
using UnsuitabilityConfig = NamedType<double, struct UnsuitabilityConfigParameter>;

template <int Dim> struct Config {
  std::array<double, Dim> values;

  template <class configType> void asureNonNegativity(configType& c)
  {
    if (c < 0) {
      c = configType { 0 };
    } else if (!std::isfinite(c)) {
      std::cout << "Masking infinity" << '\n';
      c = configType { 1 };
    } else if (c > 1) {
      c = configType { 1 };
    }
  }

  Config(LengthConfig l, HeightConfig h, UnsuitabilityConfig u)
  {
    values[0] = l;
    asureNonNegativity(values[0]);
    if constexpr (Dim >= 2) {
      values[1] = h;
      asureNonNegativity(values[1]);
    }
    if constexpr (Dim >= 3) {
      values[2] = u;
      asureNonNegativity(values[2]);
    }
  };

  Config(const std::vector<double>& values)
  {
    for (size_t i = 0; i < Dim; ++i) {
      this->values[i] = values[i];
      asureNonNegativity(this->values[i]);
    }
  }

  bool operator==(Config& other)
  {
    for (size_t i = 0; i < Dim; ++i) {
      if (values[i] != other.values[i]) {
        return false;
      }
    }
    return true;
  }

  Config integerValues() const
  {
    std::vector<double> intValues;
    for (size_t i = 0; i < Dim; ++i) {
      intValues.push_back(std::round(values[i] * 100));
    }
    return intValues;
  }
};

template <int Dim> Config<Dim> generateRandomConfig();

template <int Dim> std::ostream& operator<<(std::ostream& stream, const Config<Dim>& c);

template <int Dim> struct Route {
  Cost<Dim> costs;
  std::deque<EdgeId> edges;
};

template <int Dim> class Dijkstra {
  public:
  using GraphD = Graph<Dim>;
  using RouteD = Route<Dim>;
  using ConfigD = Config<Dim>;

  using ScalingFactor = std::array<double, Dim>;

  Dijkstra(GraphD* g, size_t nodeCount);
  Dijkstra(const Dijkstra& other) = default;
  Dijkstra(Dijkstra&& other) = default;
  virtual ~Dijkstra() noexcept = default;
  Dijkstra& operator=(const Dijkstra& other) = default;
  Dijkstra& operator=(Dijkstra&& other) = default;

  std::optional<RouteD> findBestRoute(NodePos from, NodePos to, ConfigD config);
  void calcScalingFactor(NodePos from, NodePos to, ScalingFactor& f);

  size_t pqPops = 0;

  private:
  enum class Direction { S, T };
  using QueueElem = std::tuple<NodePos, double, Direction>;
  struct QueueComparator {
    bool operator()(QueueElem left, QueueElem right);
  };
  using Queue = std::priority_queue<QueueElem, std::vector<QueueElem>, QueueComparator>;

  void clearState();

  using NodeToEdgeMap = std::unordered_map<NodePos, EdgeId>;
  RouteD buildRoute(NodePos node, NodeToEdgeMap& previousEdgeS, NodeToEdgeMap& previousEdgeT,
      NodePos from, NodePos to);

  void relaxEdges(const NodePos& node, double cost, Direction dir, Queue& heap,
      NodeToEdgeMap& previousEdge, std::vector<double>& costs);

  bool stallOnDemand(const NodePos& node, double cost, Direction dir, std::vector<double>& costs);

  double minCandidate;
  std::vector<double> costS;
  std::vector<double> costT;
  std::vector<NodePos> touchedS;
  std::vector<NodePos> touchedT;
  ConfigD config = ConfigD(LengthConfig(0), HeightConfig(0), UnsuitabilityConfig(0));
  GraphD* graph;
};

#include "dijkstra.inc"

#endif /* DIJKSTRA_H */
