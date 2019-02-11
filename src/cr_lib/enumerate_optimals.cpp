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

int FullCellId::currentId = 0;
std::vector<int> FullCellId::alive_ = std::vector<int>();
std::vector<bool> FullCellId::checked_ = std::vector<bool>();
std::vector<double> FullCellId::prio_ = std::vector<double>();

FullCellId::FullCellId()
    : id_(currentId++)
{
  alive_.push_back(1);
  checked_.push_back(false);
  prio_.push_back(-1.0);
}

FullCellId::FullCellId(const FullCellId& other)
    : id_(other.id_)
{
  alive_[id_]++;
}

FullCellId& FullCellId::operator=(const FullCellId& rhs)
{
  // Check for self-assignment!
  if (this == &rhs)
    return *this;

  alive_[id_]--;
  id_ = rhs.id_;
  alive_[id_]++;
  return *this;
}

FullCellId& FullCellId::operator=(const FullCellId&& rhs)
{
  *this = rhs;
  return *this;
}

FullCellId::FullCellId(FullCellId&& other)
{
  id_ = other.id_;
  alive_[id_]++;
}

FullCellId::~FullCellId() { alive_[id_]--; }

bool FullCellId::alive() const { return alive_[id_] > 1; }
bool FullCellId::checked() const { return checked_[id_]; }
void FullCellId::checked(bool check) { checked_[id_] = check; }
size_t FullCellId::id() const { return id_; }
double FullCellId::prio() const { return prio_[id_]; }
void FullCellId::prio(double p) const { prio_[id_] = p; }

bool FullCellId::operator==(const FullCellId& other) const { return id_ == other.id_; }

double EnumerateOptimals::compare(size_t i, size_t j)
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

Config EnumerateOptimals::findConfig(const TDS::Full_cell& f)
{
  using namespace Eigen;

  auto vert_count = f.maximal_dimension() + 1;
  Matrix<double, Dynamic, DIMENSION> A(vert_count, DIMENSION);
  VectorXd b = VectorXd::Ones(vert_count);
  int row = -1;
  for (auto vert = f.vertices_begin(); vert != f.vertices_end(); ++vert) {
    ++row;
    if (*vert == TDS::Vertex_const_handle() || tri.is_infinite(*vert)) {
      for (size_t col = 0; col < DIMENSION; ++col) {
        A(row, col) = 0;
      }
      b[row] = 0;
      continue;
    }
    int col = -1;
    auto& p = (*vert)->point();
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

void EnumerateOptimals::addToTriangulation()
{
  auto vertId = routes.size() - 1;
  auto& routeCosts = routes[vertId].costs;
  Triangulation::Point p(DIMENSION, &routeCosts.values[0], &routeCosts.values[DIMENSION]);
  auto vertex = tri.insert(p);
  auto& vertexdata = vertex->data();
  vertexdata.id = vertId;
}

void EnumerateOptimals::includeConvexHullCells(CellContainer& cont)
{
  std::vector<TDS::Full_cell_handle> handles;
  tri.incident_full_cells(tri.infinite_vertex(), std::back_inserter(handles));
  for (size_t i = 0; i < handles.size(); ++i) {
    TDS::Full_cell& c = *handles[i];
    if (c.data().checked()) {
      continue;
    }

    if (c.data().prio() < 0.0 && maxOverlap < 1.0) {
      std::vector<TDS::Vertex_handle> vertices;
      for (auto v = c.vertices_begin(); v != c.vertices_end(); ++v) {
        auto& vertex = *v;
        if (vertex == TDS::Vertex_const_handle() || tri.is_infinite(vertex)) {
          continue;
        }
        vertices.push_back(vertex);
      }

      auto result = 0;
      for (size_t i = 0; i < routes.size(); ++i) {
        for (auto& vertex : vertices) {
          auto vertId = vertex->data().id;
          if (vertId != i && compare(i, vertId) > maxOverlap) {
            ++result;
          }
        }
      }
      c.data().prio(result);
    }
    cont.emplace(c);
  }
}

std::vector<size_t> EnumerateOptimals::extract_independent_set(
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

std::tuple<std::vector<size_t>, EnumerateOptimals::Edges> EnumerateOptimals::vertex_ids_and_edges()
{
  typedef Triangulation::Face Face;
  typedef std::vector<Face> Faces;

  std::vector<size_t> vertices;
  Edges edges;

  auto get_edges = [&edges, this](auto v) {
    Faces adjacent_edges;
    tri.tds().incident_faces(v, 1, std::back_inserter(adjacent_edges));
    for (auto& f : adjacent_edges) {
      auto v0 = f.vertex(0);
      auto v1 = f.vertex(1);
      if (tri.is_infinite(v0) || tri.is_infinite(v1)) {
        continue;
      }
      auto id0 = v0->data().id;
      auto id1 = v1->data().id;
      if (id0 > id1) {
        std::swap(id0, id1);
      }
      edges.emplace_back(id0, id1);
    }
  };

  if (tri.number_of_vertices() >= DIMENSION) {
    Faces ch_edges;
    std::back_insert_iterator<Faces> out(ch_edges);
    tri.tds().incident_faces(tri.infinite_vertex(), 1, out);

    vertices.reserve(ch_edges.size());
    for (auto& f : ch_edges) {
      TDS::Vertex_handle v = f.vertex(0);
      if (tri.is_infinite(v)) {
        v = f.vertex(1);
      }
      vertices.push_back(v->data().id);
      get_edges(v);
    }
  } else {
    for (TDS::Vertex_iterator vertex = tri.vertices_begin(); vertex != tri.vertices_end();
         ++vertex) {
      if (vertex == TDS::Vertex_handle() || tri.is_infinite(*vertex)) {
        continue;
      }
      vertices.push_back(vertex->data().id);
      get_edges(vertex);
    }
  }
  std::sort(edges.begin(), edges.end());
  auto last = std::unique(edges.begin(), edges.end());
  edges.erase(last, edges.end());

  return { vertices, edges };
}

EnumerateOptimals::EnumerateOptimals(Graph* g, double maxOverlap, size_t maxRoutes)
    : g(g)
    , maxOverlap(maxOverlap)
    , maxRoutes(maxRoutes)
    , d(g->createDijkstra())
{
  for (size_t i = 0; i < DIMENSION; ++i) {
    this->important[i] = false;
  }
}

EnumerateOptimals::EnumerateOptimals(
    Graph* g, double maxOverlap, size_t maxRoutes, Important important, Slack slack)
    : g(g)
    , maxOverlap(maxOverlap)
    , maxRoutes(maxRoutes)
    , d(g->createDijkstra())
{
  for (size_t i = 0; i < DIMENSION; ++i) {
    this->important[i] = important[i];
    this->slack[i] = slack[i];
  }
}

void EnumerateOptimals::find(NodePos s, NodePos t)
{
  auto start = c::high_resolution_clock::now();
  routes.clear();
  configs.clear();
  similarities.clear();
  tri.clear();

  auto exclude = create_exclusion_set_from_important_metrics(s, t);
  d.exclude(exclude);

  run_not_important_metrics(s, t);

  Config conf(std::vector(DIMENSION, 1.0 / DIMENSION));
  auto route = d.findBestRoute(s, t, conf);

  if (route) {
    routes.push_back(std::move(*route));
    configs.push_back(std::move(conf));
    addToTriangulation();
  }
  double maxValue = *std::max_element(&factor[0], &factor[DIMENSION]);

  for (double& value : factor) {
    value = maxValue / value;
  }

  CellContainer q(compare_prio);
  bool workToDo = true;
  while (workToDo) {
    includeConvexHullCells(q);
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
        for (auto vertex = f.vertices_begin(); vertex != f.vertices_end(); ++vertex) {
          if (*vertex == TDS::Vertex_const_handle() || tri.is_infinite(*vertex)) {
            continue;
          } else {
            TDS::Vertex_handle v = *vertex;
            if (compare(v->data().id, lastRoute) == 1.0) {
              include = false;
            }
            break;
          }
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

size_t EnumerateOptimals::found_route_count() const { return routes.size(); }
size_t EnumerateOptimals::vertex_count() const { return tri.number_of_vertices(); }

std::tuple<std::vector<Route>, std::vector<Config>, EnumerateOptimals::Edges>
EnumerateOptimals::recommend_routes(bool ilp)
{

  auto [vertices, edges] = vertex_ids_and_edges();
  auto independent_set = extract_independent_set(vertices, ilp);

  std::vector<Route> routes;
  routes.reserve(independent_set.size());
  std::vector<Config> configs;
  configs.reserve(independent_set.size());

  for (auto id : independent_set) {
    routes.push_back(this->routes[vertices[id]]);

    auto c = this->configs[vertices[id]];
    for (size_t i = 0; i < DIMENSION; ++i) {
      c.values[i] /= factor[i];
    }
    auto sum = std::accumulate(&c.values[0], &c.values[DIMENSION], 0.0);

    for (size_t i = 0; i < DIMENSION; ++i) {
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

exclusion_set EnumerateOptimals::create_exclusion_set_from_important_metrics(NodePos s, NodePos t)
{
  exclusion_set exclude(g->getNodeCount(), false);
  for (size_t i = 0; i < DIMENSION; ++i) {
    if (!important[i])
      continue;

    Config conf(std::vector(DIMENSION, 0.0));
    conf.values[i] = 1.0;

    auto route = d.findBestRoute(s, t, conf);
    if (!route) {
      throw std::runtime_error(
          "No route found from " + std::to_string(s) + " to " + std::to_string(t));
    }

    d.excluded_nodes(slack[i], exclude);

    factor[i] = route->costs.values[i];
    routes.push_back(std::move(*route));
    configs.push_back(std::move(conf));
    addToTriangulation();
  }
  return exclude;
}

void EnumerateOptimals::run_not_important_metrics(NodePos s, NodePos t)
{

  for (size_t i = 0; i < DIMENSION; ++i) {
    if (important[i])
      continue;

    Config conf(std::vector(DIMENSION, 0.0));
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

std::pair<Important, Slack> EnumerateOptimals::important_metrics_to_arrays(
    std::vector<ImportantMetric> metrics)
{
  Important important;
  Slack slack;
  slack.fill(std::numeric_limits<double>::max());

  for (const auto& metric : metrics) {
    important[metric.metric] = true;
    slack[metric.metric] = metric.slack;
  }

  return std::make_pair(important, slack);
}
