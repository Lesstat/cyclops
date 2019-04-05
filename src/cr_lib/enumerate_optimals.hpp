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

#ifndef ENUMERATE_OPTIMALS_H
#define ENUMERATE_OPTIMALS_H

#include "cgaltypes.hpp"
#include "cost_triangulation.hpp"
#include "dijkstra.hpp"
#include "ilp_independent_set.hpp"
#include "prio_policy.hpp"
#include "restriction_policy.hpp"
#include "routeComparator.hpp"
#include "similarity_policy.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <queue>

auto compare_prio = [](auto left, auto right) { return left.data().prio() > right.data().prio(); };

namespace c = std::chrono;

template <int Dim, class Derived>
class DefaultsOnly : public AllFacetsPolicy<Dim>,
                     public SimilarityPolicy<Dim>,
                     public FacetPrioPolicy<Dim>,
                     public CostTriangulation<Dim, Derived> {
  private:
  friend Derived;
  DefaultsOnly() = default;
};

template <int Dim, class Derived>
class OnlyExclusion : public ThresholdPolicy<Dim>,
                      public SharingSimilarityPolicy<Dim, Derived>,
                      public FacetPrioPolicy<Dim>,
                      public CostTriangulation<Dim, Derived> {
  private:
  friend Derived;
  OnlyExclusion() = default;
};

template <int Dim, class Derived>
class SimilarityPrio : public AllFacetsPolicy<Dim>,
                       public SharingSimilarityPolicy<Dim, Derived>,
                       public SimilarityPrioPolicy<Dim, Derived>,
                       public CostTriangulation<Dim, Derived> {
  private:
  friend Derived;
  SimilarityPrio() = default;
};

template <int Dim, class Derived>
class SimilarityPrioExcludeIrrelevant : public ThresholdPolicy<Dim>,
                                        public SharingSimilarityPolicy<Dim, Derived>,
                                        public SimilarityPrioPolicy<Dim, Derived>,
                                        public CostTriangulation<Dim, Derived> {
  private:
  friend Derived;
  SimilarityPrioExcludeIrrelevant() = default;
};

template <int Dim, template <int, typename> typename Skills>
class EnumerateOptimals : public Skills<Dim, EnumerateOptimals<Dim, Skills>> {

  public:
  using CgalDim = typename CgalTypes<Dim>::CgalDim;
  using FullCell = typename CgalTypes<Dim>::FullCell;
  using TDS = typename CgalTypes<Dim>::TDS;
  const static int Dimension;

  using CellContainer = std::priority_queue<typename TDS::Full_cell,
      std::vector<typename TDS::Full_cell>, decltype(compare_prio)>;

  using Graph = Graph<Dim>;
  using Route = Route<Dim>;
  using Config = Config<Dim>;
  using Dijkstra = Dijkstra<Dim>;

  typedef std::vector<std::pair<size_t, size_t>> Edges;

  private:
  Graph* g;
  std::vector<Route> routes;
  std::vector<Config> configs;
  size_t maxRoutes;
  Dijkstra d;
  typename Dijkstra::ScalingFactor factor;

  Config findConfig(const typename TDS::Full_cell& f)
  {
    using namespace Eigen;

    auto vertices = this->cell_vertices(f);
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

  void clear()
  {
    routes.clear();
    configs.clear();
    this->tri_clear();
    this->prio_clear();
    this->sim_clear();
    this->excl_clear();
  }

  void addToTriangulation()
  {
    auto vertId = routes.size() - 1;
    this->register_route(routes[vertId]);
    auto& routeCosts = routes[vertId].costs;
    this->add_route(routeCosts, vertId);
  }

  std::vector<size_t> extract_independent_set(const std::vector<size_t>& vertices, bool ilp)
  {
    auto start = c::high_resolution_clock::now();

    std::vector<size_t> independent_set;
    if constexpr (std::is_base_of<FacetPrioPolicy<Dim>,
                      Skills<Dim, EnumerateOptimals<Dim, Skills>>>::value) {
      const auto vertex_count = vertices.size();
      independent_set.reserve(vertex_count);
      for (size_t i = 0; i < vertex_count; ++i) {
        independent_set.push_back(i);
      }

    } else {

      Edges edges;

      for (size_t i = 0; i < vertices.size(); ++i) {
        for (size_t j = i + 1; j < vertices.size(); ++j) {
          if (this->similar(vertices[i], vertices[j])) {
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
    }
    auto end = c::high_resolution_clock::now();
    recommendation_time = c::duration_cast<c::milliseconds>(end - start).count();
    return independent_set;
  }

  void run_base_configs(NodePos s, NodePos t)
  {
    for (size_t i = 0; i < Dim; ++i) {
      Config conf(std::vector(Dim, 0.0));
      conf.values[i] = 1.0;

      auto route = d.findBestRoute(s, t, conf);
      if (!route)
        continue;

      factor[i] = route->costs.values[i];

      if (std::any_of(routes.begin(), routes.end(),
              [&route](const auto& r) { return r.edges == route->edges; })) {
        continue;
      }

      routes.push_back(std::move(*route));
      configs.push_back(std::move(conf));
      addToTriangulation();
    }
  }

  public:
  size_t enumeration_time;
  size_t recommendation_time;

  EnumerateOptimals(Graph* g, size_t maxRoutes)
      : g(g)
      , maxRoutes(maxRoutes)
      , d(g->createDijkstra())
  {
  }

  void find(NodePos s, NodePos t)
  {
    auto start = c::high_resolution_clock::now();
    clear();

    run_base_configs(s, t);

    Config conf(std::vector(Dim, 1.0 / Dim));
    auto route = d.findBestRoute(s, t, conf);

    if (route && std::none_of(routes.begin(), routes.end(), [&route](const auto& r) {
          return r.edges == route->edges;
        })) {
      routes.push_back(std::move(*route));
      configs.push_back(std::move(conf));
      addToTriangulation();
    }
    double maxValue = *std::max_element(&factor[0], &factor[Dim]);

    for (double& value : factor) {
      value = maxValue / value;
    }

    std::vector<typename TDS::Full_cell> q;
    while (routes.size() < maxRoutes) {
      q.clear();
      this->get_convex_hull_cells(q);
      if (q.empty()) {
        break;
      }
      auto f = std::min_element(q.begin(), q.end(),
          [](const auto& a, const auto& b) -> bool { return a.data().prio() < b.data().prio(); });

      FullCellId& cellData = const_cast<FullCellId&>(f->data());
      cellData.checked(true);

      try {
        Config conf = findConfig(*f);
        auto route = d.findBestRoute(s, t, conf);
        if (route && std::none_of(routes.begin(), routes.end(), [route](const auto& r) {
              return r.edges == route->edges;
            })) {
          routes.push_back(std::move(*route));
          configs.push_back(std::move(conf));
          addToTriangulation();
        }

      } catch (std::runtime_error& e) {
      }
    }

    auto end = c::high_resolution_clock::now();

    enumeration_time = c::duration_cast<c::milliseconds>(end - start).count();
  }

  size_t found_route_count() const { return routes.size(); }
  size_t vertex_count() const { return this->number_of_vertices(); }

  std::tuple<std::vector<Route>, std::vector<Config>, EnumerateOptimals::Edges> recommend_routes(
      bool ilp)
  {
    auto [vertices, edges] = this->vertex_ids_and_edges();
    auto independent_set = extract_independent_set(vertices, ilp);

    std::vector<Route> routes;
    routes.reserve(independent_set.size());
    std::vector<Config> configs;
    configs.reserve(independent_set.size());

    for (auto id : independent_set) {
      auto v = vertices.at(id);
      auto& r = this->routes.at(v);
      routes.push_back(r);

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
  const Route& route(size_t i) const { return routes[i]; }
};

#endif /* ENUMERATE_OPTIMALS_H */
