#include "dijkstra.hpp"

Dijkstra::Dijkstra(Graph const& g, size_t nodeCount)
    : cost_s(nodeCount)
    , cost_t(nodeCount)
    , graph(g)
{
}
Dijkstra::Dijkstra(const Dijkstra& other)
    : graph(other.graph)
{
}

// Dijkstra::Dijkstra(Dijkstra&& other) noexcept
// {
//   graph = other.graph;
// }

Dijkstra::~Dijkstra() noexcept {}

//Dijkstra& Dijkstra::operator=(const Dijkstra& other) {}
//Dijkstra& Dijkstra::operator=(Dijkstra&& other) noexcept {}

void Dijkstra::clear_state() {}
