/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2019  Florian Barth

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

#include "enumerate_optimals.hpp"
#include "url_parsing.hpp"

template <int Dim, class ExclP, class PrioP>
double EnumerateOptimals<Dim, ExclP, PrioP>::compare(size_t i, size_t j)
{
  if (i > j) {
    std::swap(i, j);
  }
  std::pair index = { i, j };
  if (similarities.count(index) > 0) {
    return similarities[index];
  }
  auto similarity = calculateSharing(routes[i], routes[j]);
  similarities[index] = similarity;
  return similarity;
}

template <int Dim, class ExclP, class PrioP>
Config<Dim> EnumerateOptimals<Dim, ExclP, PrioP>::findConfig(const typename TDS::Full_cell& f)
{
  using namespace Eigen;

  auto vertices = tri.cell_vertices(f);
  auto vert_count = vertices.size();
  Matrix<double, Dynamic, Dim> A(vert_count, Dim);
  VectorXd b = VectorXd::Ones(vert_count);
  int row = -1;
  for (const auto& vert : vertices) {
    ++row;
    int col = -1;
    auto& p = vert->point();
    for (auto cord = p.cartesian_begin(); cord != p.cartesian_end(); ++cord) {
      ++col;
      A(row, col) = *cord;
    }
  }
  VectorXd solution = A.fullPivHouseholderQr().solve(b);
  std::vector<double> conf_values;

  double sum = 0;
  for (long i = 0; i < solution.size(); ++i) {
    if (solution[i] < -0.001)
      throw std::runtime_error("solution component too negative pre normalization");
    sum += solution[i];
    conf_values.push_back(solution[i]);
  }

  if (sum == 0) {
    throw std::runtime_error("All components zero");
  }

  for (auto& val : conf_values) {
    val /= sum;
    if (val < -0.001)
      throw std::runtime_error("solution component too negative after normalization");
  }

  return Config(conf_values);
}

template <int Dim, class ExclP, class PrioP>
void EnumerateOptimals<Dim, ExclP, PrioP>::addToTriangulation()
{
  auto vertId = routes.size() - 1;

  policy.register_route(routes[vertId]);

  auto& routeCosts = routes[vertId].costs;
  tri.add_route(routeCosts, vertId);
}

template <int Dim, class ExclP, class PrioP>
std::vector<size_t> EnumerateOptimals<Dim, ExclP, PrioP>::extract_independent_set(
    const std::vector<size_t>& vertices, bool ilp)
{

  auto start = c::high_resolution_clock::now();

  std::vector<size_t> independent_set;
  if (maxOverlap < 1.0) {
    Edges edges;

    for (size_t i = 0; i < vertices.size(); ++i) {
      for (size_t j = i + 1; j < vertices.size(); ++j) {
        if (compare(vertices[i], vertices[j]) >= maxOverlap) {
          edges.emplace_back(i, j);
        }
      }
    }
    if (ilp) {
      independent_set = find_independent_set(vertices.size(), edges);
    } else {
      duplicate_edges(edges);
      independent_set = greedy_independent_set(vertices.size(), edges);
    }
  } else {
    for (size_t i = 0; i < vertices.size(); ++i) {
      independent_set.push_back(i);
    }
  }
  auto end = c::high_resolution_clock::now();
  recommendation_time = c::duration_cast<c::milliseconds>(end - start).count();
  return independent_set;
}

template <int Dim, class ExclP, class PrioP>
EnumerateOptimals<Dim, ExclP, PrioP>::EnumerateOptimals(
    Graph* g, double maxOverlap, size_t maxRoutes, PrioP prio, ExclP excl)
    : g(g)
    , maxOverlap(maxOverlap)
    , maxRoutes(maxRoutes)
    , d(g->createDijkstra())
    , policy(excl)
    , prio_pol(prio)
{
  prio_pol.set_pointers(&routes, &configs, [this](size_t i, size_t j) { return compare(i, j); });
}

template <int Dim, class ExclP, class PrioP>
void EnumerateOptimals<Dim, ExclP, PrioP>::find(NodePos s, NodePos t)
{
  auto start = c::high_resolution_clock::now();
  routes.clear();
  configs.clear();
  similarities.clear();
  tri.clear();

  run_base_configs(s, t);

  Config conf(std::vector(Dim, 1.0 / Dim));
  auto route = d.findBestRoute(s, t, conf);

  if (route) {
    routes.push_back(std::move(*route));
    configs.push_back(std::move(conf));
    addToTriangulation();
  }
  double maxValue = *std::max_element(&factor[0], &factor[Dim]);

  for (double& value : factor) {
    value = maxValue / value;
  }

  CellContainer q(compare_prio);
  bool workToDo = true;
  while (workToDo) {
    tri.get_convex_hull_cells(q, policy, prio_pol);
    workToDo = false;
    while (!q.empty() && routes.size() < maxRoutes) {
      auto f = q.top();
      q.pop();
      FullCellId& cellData = const_cast<FullCellId&>(f.data());

      if (!cellData.alive() || cellData.checked()) {
        continue;
      }
      cellData.checked(true);

      try {
        Config conf = findConfig(f);
        auto route = d.findBestRoute(s, t, conf);
        if (!route)
          continue;

        routes.push_back(std::move(*route));
        configs.push_back(std::move(conf));

        auto lastRoute = routes.size() - 1;
        bool include = true;

        for (auto& vertex : tri.cell_vertices(f)) {
          if (compare(vertex->data().id, lastRoute) == 1.0) {
            include = false;
          }
          break;
        }

        if (include) {
          addToTriangulation();
          workToDo = true;
        }

      } catch (std::runtime_error& e) {
      }
    }
  }

  auto end = c::high_resolution_clock::now();

  enumeration_time = c::duration_cast<c::milliseconds>(end - start).count();
}

template <int Dim, class ExclP, class PrioP>
size_t EnumerateOptimals<Dim, ExclP, PrioP>::found_route_count() const
{
  return routes.size();
}
template <int Dim, class ExclP, class PrioP>
size_t EnumerateOptimals<Dim, ExclP, PrioP>::vertex_count() const
{
  return tri.number_of_vertices();
}

template <int Dim, class ExclP, class PrioP>
std::tuple<std::vector<Route<Dim>>, std::vector<Config<Dim>>,
    typename EnumerateOptimals<Dim, ExclP, PrioP>::Edges>
EnumerateOptimals<Dim, ExclP, PrioP>::recommend_routes(bool ilp)
{
  auto [vertices, edges] = tri.vertex_ids_and_edges(routes, policy);
  auto independent_set = extract_independent_set(vertices, ilp);

  std::vector<Route> routes;
  routes.reserve(independent_set.size());
  std::vector<Config> configs;
  configs.reserve(independent_set.size());

  for (auto id : independent_set) {
    routes.push_back(this->routes[vertices[id]]);

    auto c = this->configs[vertices[id]];
    for (size_t i = 0; i < Dim; ++i) {
      c.values[i] /= factor[i];
    }
    auto sum = std::accumulate(&c.values[0], &c.values[Dim], 0.0);

    for (size_t i = 0; i < Dim; ++i) {
      c.values[i] /= sum;
    }

    configs.push_back(std::move(c));
  }

  std::map<size_t, size_t> id_to_pos;
  for (size_t i = 0; i < independent_set.size(); ++i) {
    id_to_pos[independent_set[i]] = i;
  }
  Edges final_edges;
  for (auto& e : edges) {
    try {
      final_edges.emplace_back(id_to_pos.at(e.first), id_to_pos.at(e.second));
    } catch (std::out_of_range& e) {
      continue;
    }
  }
  if (routes.empty()) {
    routes.push_back(this->routes.front());
    configs.push_back(this->configs.front());
  }

  return { routes, configs, edges };
}

template <int Dim, class ExclP, class PrioP>
void EnumerateOptimals<Dim, ExclP, PrioP>::run_base_configs(NodePos s, NodePos t)
{
  for (size_t i = 0; i < Dim; ++i) {
    Config conf(std::vector(Dim, 0.0));
    conf.values[i] = 1.0;

    auto route = d.findBestRoute(s, t, conf);
    if (!route) {
      continue;
    }

    factor[i] = route->costs.values[i];
    routes.push_back(std::move(*route));
    configs.push_back(std::move(conf));
    addToTriangulation();
  }
}
