
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
  Dijkstra(const Graph& g, size_t nodeCount);
  Dijkstra(const Dijkstra& other);
  Dijkstra(Dijkstra&& other) noexcept;
  virtual ~Dijkstra() noexcept;

  Dijkstra& operator=(const Dijkstra& other);
  Dijkstra& operator=(Dijkstra&& other) noexcept;

  std::optional<Route> findBestRoute(NodeId from, NodeId to, Config config);

  private:
  void clearState();

  using NodeToEdgeMap = std::unordered_map<NodeId, EdgeId>;
  Route buildRoute(NodeId node, NodeToEdgeMap previousEdgeS, NodeToEdgeMap previousEdgeT, NodeId from, NodeId to);
  std::vector<double> costS;
  std::vector<double> costT;
  std::vector<NodeId> touchedS;
  std::vector<NodeId> touchedT;
  const Graph& graph;
};

#endif /* DIJKSTRA_H */
