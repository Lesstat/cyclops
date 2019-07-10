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
#ifndef GRID_H
#define GRID_H

#include "namedType.hpp"

using NodePos = NamedType<uint32_t, struct NodePosParameter>;
using Lat = NamedType<double, struct LatParameter>;
using Lng = NamedType<double, struct LngParameter>;

class Node;

struct PositionalNode {
  Lat lat;
  Lng lng;
  NodePos pos;

  PositionalNode(Lat lat, Lng lng, NodePos pos)
      : lat(lat)
      , lng(lng)
      , pos(pos)
  {
  }
};

struct BoundingBox {
  Lat lat_min = Lat(std::numeric_limits<double>::max());
  Lat lat_max = Lat(std::numeric_limits<double>::min());
  Lng lng_min = Lng(std::numeric_limits<double>::max());
  Lng lng_max = Lng(std::numeric_limits<double>::min());

  void addNode(PositionalNode& n)
  {
    if (n.lat < lat_min) {
      lat_min = n.lat;
    }
    if (n.lat > lat_max) {
      lat_max = n.lat;
    }
    if (n.lng < lng_min) {
      lng_min = n.lng;
    }
    if (n.lng > lng_max) {
      lng_max = n.lng;
    }
  }

  bool contains_point(Lat lat, Lng lng)
  {
    return lat_min <= lat && lat <= lat_max && lng_min <= lng && lng <= lng_max;
  }
};

class Grid {
  public:
  Grid() = delete;
  Grid(const std::vector<Node>& nodes, long sideLength = 100);
  Grid(const Grid& other) = default;
  Grid(Grid&& other) noexcept = default;
  virtual ~Grid() noexcept = default;
  Grid& operator=(const Grid& other) = default;
  Grid& operator=(Grid&& other) noexcept = default;

  std::optional<NodePos> findNextNode(Lat lat, Lng lng);
  BoundingBox bounding_box();

  protected:
  private:
  size_t coordsToIndex(Lat lat, Lng lng);

  BoundingBox bBox;
  std::vector<PositionalNode> nodes;
  std::vector<size_t> offset;
  long sideLength;
};

double haversine_distance(const PositionalNode& a, const PositionalNode& b);
#endif /* GRID_H */
