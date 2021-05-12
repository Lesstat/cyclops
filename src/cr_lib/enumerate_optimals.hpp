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
#include "dijkstra_thread.hpp"
#include "ilp_independent_set.hpp"
#include "prio_policy.hpp"
#include "restriction_policy.hpp"
#include "routeComparator.hpp"
#include "similarity_policy.hpp"

#include "ThreadPool.h"
#include <Eigen/Dense>

#include <chrono>
#include <queue>

const size_t THREAD_COUNT
    = std::thread::hardware_concurrency() > 0 ? std::thread::hardware_concurrency() - 1 : 1;

template <int Dim> Config<Dim> find_equal_cost_config(const std::vector<Cost<Dim>>& costs)
{
  using namespace Eigen;

  auto vert_count = costs.size();
  Matrix<double, Dynamic, Dim> A(vert_count, Dim);
  VectorXd b = VectorXd::Ones(vert_count);
  int row = -1;
  for (const auto& cost : costs) {
    ++row;
    int col = -1;
    for (const auto& c : cost.values) {
      ++col;
      A(row, col) = c;
    }
  }
  VectorXd solution = A.fullPivHouseholderQr().solve(b);
  std::vector<double> conf_values;

  double sum = 0;
  for (long i = 0; i < solution.size(); ++i) {
    solution[i] = std::max(0.0, solution[i]);
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

  return Config<Dim>(conf_values);
}

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
                      public SimilarityPolicy<Dim>,
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

  static auto compare_prio
      = [](auto left, auto right) { return left.data().prio() > right.data().prio(); };
  using CellContainer = std::priority_queue<typename TDS::Full_cell,
      std::vector<typename TDS::Full_cell>, decltype(compare_prio)>;

  using Graph = Graph<Dim>;
  using Route = Route<Dim>;
  using Config = Config<Dim>;
  using Dijkstra = Dijkstra<Dim>;
  using Cost = Cost<Dim>;

  typedef std::vector<std::pair<size_t, size_t>> Edges;

  private:
  Graph* g;
  std::vector<Route> routes;
  std::vector<Config> configs;
  size_t maxRoutes;

  size_t pending_requests = 0;
  std::vector<DijkstraThread<Dim>> d;
  MultiQueue<RoutingRequest<Dim>> req_queue;
  MultiQueue<RoutingResult<Dim>> res_queue;
  typename Dijkstra::ScalingFactor factor;

  Config findConfig(const std::vector<Cost>& costs) { return find_equal_cost_config(costs); }

  void clear()
  {
    routes.clear();
    configs.clear();
    this->tri_clear();
    this->prio_clear();
    this->sim_clear();
    this->excl_clear();
  }

  void schedule_routing(RoutingRequest<Dim> r)
  {
    req_queue.send(r);
    ++pending_requests;
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
      schedule_routing(RoutingRequest<Dim>(s, t, conf));
    }

    Config conf(std::vector(Dim, 1.0 / Dim));
    schedule_routing(RoutingRequest<Dim>(s, t, conf));

    for (size_t i = 0; i < Dim + 1; ++i) {
      process_routing_result(res_queue.receive());
    }

    for (size_t i = 0; i < configs.size(); ++i) {
      for (size_t j = 0; j < Dim; ++j) {
        if (configs[i].values[j] == 1) {
          factor[j] = routes[i].costs.values[j];
          break;
        }
      }
    }
  }

  std::optional<size_t> process_routing_result(RoutingResult<Dim>&& result)
  {
    --pending_requests;

    auto& route = result.route;
    auto& conf = result.c;

    if (!route || std::any_of(routes.begin(), routes.end(), [&route](const auto& r) {
          return r.edges == route->edges;
        })) {
      return {};
    }

    routes.push_back(std::move(*route));
    configs.push_back(std::move(conf));
    addToTriangulation();
    return routes.size() - 1;
  }

  public:
  size_t enumeration_time;
  size_t recommendation_time;

  EnumerateOptimals(Graph* g, size_t maxRoutes)
      : g(g)
      , maxRoutes(maxRoutes)
  {

    d.reserve(THREAD_COUNT);
    for (size_t i = 0; i < THREAD_COUNT; ++i) {
      d.emplace_back(req_queue, g->createDijkstra(), res_queue);
    }

    for (auto& dij : d) {
      dij.run();
    }
  }

  ~EnumerateOptimals()
  {
    using namespace std::chrono_literals;
    req_queue.close();
    res_queue.close();
  }

  void find(NodePos s, NodePos t)
  {
    auto start = c::high_resolution_clock::now();
    clear();

    run_base_configs(s, t);

    double maxValue = *std::max_element(&factor[0], &factor[Dim]);

    for (double& value : factor) {
      value = maxValue / value;
    }
    while (routes.size() <= Dim) {
      std::vector<Cost> costs;
      std::transform(routes.begin(), routes.end(), std::back_inserter(costs),
          [](const auto& r) { return r.costs; });
      try {
        schedule_routing(RoutingRequest<Dim>(s, t, findConfig(costs)));

        auto index = process_routing_result(res_queue.receive());
        if (!index) {
          std::cerr << "i'm giving up with " << routes.size() << " routes." << '\n';
          break;
        }
      } catch (std::runtime_error& e) {
        std::cout << "error: " << e.what() << "\n";
        break;
      }
    }

    std::vector<typename TDS::Full_cell> q;
    while (routes.size() < maxRoutes) {
      q.clear();
      this->get_convex_hull_cells(q);
      if (q.empty() && pending_requests == 0) {
        break;
      }

      while (!q.empty()) {

        auto f = q.back();
        q.pop_back();

        FullCellId& cellData = const_cast<FullCellId&>(f.data());
        cellData.checked(true);
        try {
          std::vector<Cost> costs;
          for (const auto& v : this->cell_vertices(f)) {
            costs.push_back(routes[v->data().id].costs);
          }

          schedule_routing(RoutingRequest<Dim>(s, t, findConfig(costs)));

        } catch (std::runtime_error& e) {
          std::cerr << "error: " << e.what() << "\n";
        }
      }

      if (pending_requests > 0) {
        process_routing_result(res_queue.receive()); // receive at least one afer each round
      }
      RoutingResult<Dim> res;
      while (res_queue.try_receive(res)) {
        process_routing_result(std::move(res));
      }
    }

    auto end = c::high_resolution_clock::now();

    enumeration_time = c::duration_cast<c::milliseconds>(end - start).count();
  }

  size_t found_route_count() const { return routes.size(); }
  size_t vertex_count() const { return this->number_of_vertices(); }

  std::tuple<std::vector<Route>, std::vector<Config>> recommend_routes(bool ilp)
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
    if (routes.empty() && !this->routes.empty()) {
      routes.push_back(this->routes.front());
      configs.push_back(this->configs.front());
    }

    return { routes, configs };
  }
  const Route& route(size_t i) const { return routes[i]; }
};

#endif /* ENUMERATE_OPTIMALS_H */
