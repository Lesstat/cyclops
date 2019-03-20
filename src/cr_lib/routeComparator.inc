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
#include "routeComparator.hpp"

template <int Dim>
DiscreteFrechet<Dim>::DiscreteFrechet(const Route& reference, const Route& other, const Graph& g)
{
  using Edge = Edge<Dim>;
  for (const auto& e : reference.edges) {
    this->reference.push_back(g.getNode(Edge::sourcePos(e)));
  }
  if (!reference.edges.empty()) {
    const auto& lastEdge = reference.edges[reference.edges.size() - 1];
    this->reference.push_back(g.getNode(Edge::destPos(lastEdge)));
  }
  for (const auto& e : other.edges) {
    this->other.push_back(g.getNode(Edge::sourcePos(e)));
  }
  if (!other.edges.empty()) {
    const auto& lastEdge = other.edges[other.edges.size() - 1];
    this->other.push_back(g.getNode(Edge::destPos(lastEdge)));
  }

  ca = std::vector(reference.edges.size() - 1, std::vector<double>(other.edges.size() - 1, -1));
}

template <int Dim> double DiscreteFrechet<Dim>::calculate()
{
  return c(ca.size() - 1, ca[0].size() - 1);
}

template <int Dim> double DiscreteFrechet<Dim>::c(int i, int j)
{

  if (ca[i][j] > -1) {
    return ca[i][j];
  }
  if (i == 0 && j == 0) {
    ca[i][j] = haversine_distance(reference[i], other[j]);
  } else if (i > 0 && j == 0) {
    ca[i][j] = std::max(c(i - 1, 0), haversine_distance(reference[i], other[0]));
  } else if (i == 0 && j > 0) {
    ca[i][j] = std::max(c(0, j - 1), haversine_distance(reference[0], other[j]));
  } else if (i > 0 && j > 0) {
    double var1 = c(i - 1, j);
    double var2 = c(i - 1, j - 1);
    double var3 = c(i, j - 1);

    double min = std::min(var1, var2);
    min = std::min(var3, min);

    ca[i][j] = std::max(min, haversine_distance(reference[i], other[j]));
  } else {
    ca[i][j] = std::numeric_limits<double>::max();
  }
  return ca[i][j];
}

template <int Dim>
double calculateSharing(const Route<Dim>& referenceRoute, const Route<Dim>& otherRoute)
{
  std::unordered_set<EdgeId> edges(referenceRoute.edges.begin(), referenceRoute.edges.end());

  uint32_t sharedLength = std::count_if(otherRoute.edges.begin(), otherRoute.edges.end(),
      [&edges](const auto& id) { return edges.find(id) != edges.end(); });

  size_t maxLength = std::max(referenceRoute.edges.size(), otherRoute.edges.size());
  return (double)sharedLength / maxLength;
}
