
#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "graph.hpp"
#include <vector>

class Dijkstra {
  public:
  Dijkstra(const Graph& g, size_t nodeCount);
  Dijkstra(const Dijkstra& other);
  Dijkstra(Dijkstra&& other) noexcept;
  virtual ~Dijkstra() noexcept;

  Dijkstra& operator=(const Dijkstra& other);
  Dijkstra& operator=(Dijkstra&& other) noexcept;

  private:
  void clear_state();

  std::vector<Cost> cost_s;
  std::vector<Cost> cost_t;
  std::vector<NodeId> touched_s;
  std::vector<NodeId> touched_t;
  const Graph& graph;
};

#endif /* DIJKSTRA_H */
