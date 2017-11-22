#include "dijkstra.hpp"

double max = std::numeric_limits<double>::max();
Cost maxCost{ Length(max), Height(max), Unsuitability(max) };

Dijkstra::Dijkstra(Graph const& g, size_t nodeCount)
    : costS(nodeCount, maxCost)
    , costT(nodeCount, maxCost)
    , graph(g)
{
}
Dijkstra::Dijkstra(const Dijkstra& other)
    : costS(other.costS)
    , costT(other.costT)
    , touchedS(other.touchedS)
    , touchedT(other.touchedT)
    , graph(other.graph)
{
}

Dijkstra::Dijkstra(Dijkstra&& other) noexcept
    : costS(std::move(other.costS))
    , costT(std::move(other.costT))
    , touchedS(std::move(other.touchedS))
    , touchedT(std::move(other.touchedT))
    , graph(std::move(other.graph))
{
}

Dijkstra::~Dijkstra() noexcept {}

//Dijkstra& Dijkstra::operator=(const Dijkstra& other) {}
//Dijkstra& Dijkstra::operator=(Dijkstra&& other) noexcept {}

void Dijkstra::clearState() {}
