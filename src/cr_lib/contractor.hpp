/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2017  Florian Barth

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
#ifndef CONTRACTOR_H
#define CONTRACTOR_H
#include "dijkstra.hpp"
#include <set>

class MultiQueue;

class Contractor {

  public:
  Contractor() = default;
  Contractor(const Contractor& other) = default;
  Contractor(Contractor&& other) = default;
  virtual ~Contractor() noexcept = default;
  Contractor& operator=(const Contractor& other) = delete;
  Contractor& operator=(Contractor&& other) = delete;

  static Edge createShortcut(const Edge& e1, const Edge& e2);

  std::pair<bool, std::optional<Route>> isShortestPath(
      Dijkstra& d, const EdgeId& startEdgeId, const EdgeId& destEdgeId, const Config& conf);

  void contract(MultiQueue& queue, Graph& g);
  Graph contract(Graph& g);
  Graph mergeWithContracted(Graph& g);
  Graph contractCompletely(Graph& g);

  std::set<NodePos> independentSet(const Graph& g);
  std::set<NodePos> reduce(std::set<NodePos>& set, const Graph& g);
  std::set<NodePos> reduce(std::set<NodePos>&& set, const Graph& g) { return reduce(set, g); };

  protected:
  private:
  std::optional<Dijkstra> dijkstra;
  std::optional<Route> foundRoute;
  size_t level = 0;
  std::vector<Node> contractedNodes;
  std::vector<Edge> contractedEdges;
};

#endif /* CONTRACTOR_H */
