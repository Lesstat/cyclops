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
#ifndef DIJKSTRA_THREAD_H
#define DIJKSTRA_THREAD_H

#include "dijkstra.hpp"
#include "multiqueue.hpp"

#include <optional>
#include <thread>

template <int D> struct RoutingRequest {
  NodePos from;
  NodePos to;
  Config<D> c;
  RoutingRequest(NodePos from, NodePos to, Config<D> c)
      : from(from)
      , to(to)
      , c(c)
  {
  }
};

template <int D> struct RoutingResult {
  Config<D> c;
  std::optional<Route<D>> route;
  RoutingResult(Config<D> c, std::optional<Route<D>> route)
      : c(c)
      , route(route)
  {
  }

  RoutingResult()
      : c(Config<D>(std::vector<double>(D, 0.0)))
      , route({})
  {
  }
};

template <int D> class DijkstraThread {
  public:
  DijkstraThread() = delete;
  DijkstraThread(const DijkstraThread& other) = default;
  DijkstraThread(DijkstraThread&& other) noexcept = default;
  virtual ~DijkstraThread() noexcept { t.join(); }
  DijkstraThread& operator=(const DijkstraThread& other) = default;
  DijkstraThread& operator=(DijkstraThread&& other) noexcept = default;

  DijkstraThread(
      MultiQueue<RoutingRequest<D>>& input, Dijkstra<D> d, MultiQueue<RoutingResult<D>>& output)
      : input(input)
      , output(output)
      , d(std::move(d))
  {
  }

  void run()
  {

    t = std::thread([this]() {
      while (!input.closed()) {
        try {
          auto req = input.receive();
          RoutingResult<D> res;
          res.route = d.findBestRoute(req.from, req.to, req.c);
          res.c = req.c;
          output.send(res);
        } catch (std::exception&) {
        }
      }
    });
  }

  protected:
  private:
  MultiQueue<RoutingRequest<D>>& input;
  MultiQueue<RoutingResult<D>>& output;
  Dijkstra<D> d;
  std::thread t;
};

#endif /* DIJKSTRA_THREAD_H */
