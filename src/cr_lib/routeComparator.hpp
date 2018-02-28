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
#ifndef ROUTECOMPARATOR_H
#define ROUTECOMPARATOR_H

#include "dijkstra.hpp"

double calculateSharing(const Route& referenceRoute, const Route& otherRoute)
{
  std::set<EdgeId> edges;
  std::transform(begin(referenceRoute.edges), end(referenceRoute.edges),
      std::inserter(edges, begin(edges)), [](const auto& edge) { return edge.getId(); });
  size_t sharedCounter = 0;
  for (const auto& edge : otherRoute.edges) {
    if (edges.find(edge.getId()) != edges.end()) {
      ++sharedCounter;
    }
  }

  size_t maxEdges = std::max(referenceRoute.edges.size(), otherRoute.edges.size());
  return static_cast<double>(sharedCounter) / maxEdges;
}

#endif /* ROUTECOMPARATOR_H */
