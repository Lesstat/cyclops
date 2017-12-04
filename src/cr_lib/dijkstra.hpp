
#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "graph.hpp"
#include <deque>
#include <vector>

struct Config {
  Length length;
  Height height;
  Unsuitability unsuitability;

  Config(Length l, Height h, Unsuitability u)
      : length(l)
      , height(h)
      , unsuitability(u)
  {
  }
};

struct Route {
  Cost costs;
  std::deque<Node> nodes;
};

class Dijkstra {
  public:
  Dijkstra(const Graph& g, size_t nodeCount);
  Dijkstra(const Dijkstra& other);
  Dijkstra(Dijkstra&& other) noexcept;
  virtual ~Dijkstra() noexcept;

  Dijkstra& operator=(const Dijkstra& other);
  Dijkstra& operator=(Dijkstra&& other) noexcept;

  Route findBestRoute(NodeId from, NodeId to, Config config);

  private:
  void clearState();

  std::vector<Cost> costS;
  std::vector<Cost> costT;
  std::vector<NodeId> touchedS;
  std::vector<NodeId> touchedT;
  const Graph& graph;
};

#endif /* DIJKSTRA_H */
