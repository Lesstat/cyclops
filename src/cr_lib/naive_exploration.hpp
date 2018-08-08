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

#ifndef NAIVE_EXPLORATION_H
#define NAIVE_EXPLORATION_H
#include "dijkstra.hpp"

inline std::vector<Route> naiveExploration(Dijkstra& d, NodePos from, NodePos to, double epsilon)
{
  std::vector<Route> result;
  const int N = std::round(1 / epsilon);
  int count = 0;
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      double alpha_1 = epsilon * i;
      double alpha_2 = epsilon * j;
      if (alpha_1 + alpha_2 > 1.0)
        continue;
      double alpha_3 = 1 - alpha_1 - alpha_2;
      count++;
      Config c{ LengthConfig{ alpha_1 }, HeightConfig{ alpha_2 }, UnsuitabilityConfig{ alpha_3 } };
      result.push_back(d.findBestRoute(from, to, c).value());
    }
  }
  return result;
}

#endif /* NAIVE_EXPLORATION_H */
