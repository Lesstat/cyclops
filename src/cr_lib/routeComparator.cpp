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

DiscreteFrechet::DiscreteFrechet(const Route& reference, const Route& other, const Graph& g)
{

  std::unordered_set<NodeId> nodeIds;
  std::transform(reference.edges.begin(), reference.edges.end(),
      std::inserter(nodeIds, begin(nodeIds)), [](const auto& e) { return e.getSourceId(); });
  std::transform(other.edges.begin(), other.edges.end(), std::inserter(nodeIds, begin(nodeIds)),
      [](const auto& e) { return e.getDestId(); });

  auto idToPos = g.getNodePosByIds(nodeIds);

  refNodes.reserve(reference.edges.size());
  for (const auto& edge : reference.edges) {
    refNodes.push_back(idToPos[edge.getSourceId()]);
  }
  refNodes.push_back(idToPos[reference.edges[reference.edges.size() - 1].getDestId()]);

  otherNodes.reserve(other.edges.size());
  for (const auto& edge : other.edges) {
    otherNodes.push_back(idToPos[edge.getSourceId()]);
  }
  otherNodes.push_back(idToPos[other.edges[other.edges.size() - 1].getDestId()]);

  ca = std::vector(refNodes.size() - 1, std::vector<double>(otherNodes.size() - 1, -1));
}

double DiscreteFrechet::calculate() { return c(ca.size() - 1, ca[0].size() - 1); }

double haversine_distance(const Node& a, const Node& b)
{
  const double EARTH_RADIUS = 6371007.2;
  const double RADIANS_CONVERSION = M_PI / 180;
  using namespace std;

  double theta1 = a.lat() * RADIANS_CONVERSION;
  double theta2 = b.lat() * RADIANS_CONVERSION;
  double deltaTheta = (b.lat() - a.lat()) * RADIANS_CONVERSION;
  double deltaLambda = (b.lng() - a.lng()) * RADIANS_CONVERSION;
  double e = pow(sin(deltaTheta / 2.0), 2)
      + std::cos(theta1) * std::cos(theta2) * pow(sin(deltaLambda / 2.0), 2);
  double c = 2.0 * asin(sqrt(e));
  return EARTH_RADIUS * c;
}

double DiscreteFrechet::c(int i, int j)
{

  if (ca[i][j] > -1) {
    return ca[i][j];
  }
  if (i == 0 && j == 0) {
    ca[i][j] = haversine_distance(*refNodes[i], *otherNodes[j]);
  } else if (i > 0 && j == 0) {
    ca[i][j] = std::max(c(i - 1, 0), haversine_distance(*refNodes[i], *otherNodes[0]));
  } else if (i == 0 && j > 0) {
    ca[i][j] = std::max(c(0, j - 1), haversine_distance(*refNodes[0], *otherNodes[j]));
  } else if (i > 0 && j > 0) {
    double var1 = c(i - 1, j);
    double var2 = c(i - 1, j - 1);
    double var3 = c(i, j - 1);

    double min = std::min(var1, var2);
    min = std::min(var3, min);

    ca[i][j] = std::max(min, haversine_distance(*refNodes[i], *otherNodes[j]));
  } else {
    ca[i][j] = std::numeric_limits<double>::max();
  }
  return ca[i][j];
}

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
