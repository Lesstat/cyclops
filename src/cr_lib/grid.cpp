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
#include "grid.hpp"
#include <cmath>

double haversine_distance(const PositionalNode& a, const PositionalNode& b)
{
  const double EARTH_RADIUS = 6371007.2;
  const double RADIANS_CONVERSION = M_PI / 180;
  using namespace std;

  double theta1 = a.lat * RADIANS_CONVERSION;
  double theta2 = b.lat * RADIANS_CONVERSION;
  double deltaTheta = (b.lat - a.lat) * RADIANS_CONVERSION;
  double deltaLambda = (b.lng - a.lng) * RADIANS_CONVERSION;
  double e
      = pow(sin(deltaTheta / 2.0), 2) + cos(theta1) * cos(theta2) * pow(sin(deltaLambda / 2.0), 2);
  double c = 2.0 * asin(sqrt(e));
  return EARTH_RADIUS * c;
}

Grid::Grid(const std::vector<Node>& nodes)
{
  for (size_t i = 0; i < nodes.size(); ++i) {
    const auto& node = nodes[i];
    this->nodes.emplace_back(node.lat(), node.lng(), NodePos{ i });
  }

  std::for_each(
      this->nodes.begin(), this->nodes.end(), [this](auto& pos) { this->bBox.addNode(pos); });

  std::sort(this->nodes.begin(), this->nodes.end(), [this](const auto& a, const auto& b) {
    return this->coordsToIndex(a.lat, a.lng) < coordsToIndex(b.lat, b.lng);
  });
  offset = std::vector<size_t>(sideLength * sideLength + 1, 0);

  size_t current = 0;
  for (size_t i = 0; i < this->nodes.size(); ++i) {
    const auto& node = this->nodes[i];
    size_t newIndex = coordsToIndex(node.lat, node.lng);
    if (newIndex != current) {
      for (size_t j = current + 1; j < newIndex + 1; ++j) {
        offset[j] = i;
      }
      current = newIndex;
    }
  }
  for (size_t i = current + 1; i < offset.size(); ++i) {
    offset[i] = this->nodes.size();
  }
  offset[offset.size() - 1] = this->nodes.size();
}

std::optional<NodePos> Grid::findNextNode(Lat lat, Lng lng)
{
  PositionalNode target{ lat, lng, NodePos{ 0 } };
  size_t center = coordsToIndex(lat, lng);
  // size_t radius = 0;

  std::optional<double> minDist = {};
  std::optional<size_t> minIndex = {};

  for (size_t i = offset[center]; i < offset[center + 1]; ++i) {
    const auto& node = nodes[i];
    double dist = haversine_distance(target, node);
    if (!minDist || dist < minDist) {
      minDist = dist;
      minIndex = i;
    }
  }

  if (minIndex) {
    return NodePos{ minIndex.value() };
  }
  return {};
}

size_t Grid::coordsToIndex(Lat lat, Lng lng)
{
  double cell_width = (bBox.lat_max - bBox.lat_min) / sideLength;
  double cell_height = (bBox.lng_max - bBox.lng_min) / sideLength;
  double lat_dif = lat - bBox.lat_min;
  double lng_dif = lng - bBox.lng_min;
  size_t x = (lat_dif / cell_width);
  size_t y = (lng_dif / cell_height);
  if (x == sideLength) {
    x -= 1;
  }
  if (y == sideLength) {
    y -= 1;
  }
  return NodePos{ y * (sideLength) + x };
}
