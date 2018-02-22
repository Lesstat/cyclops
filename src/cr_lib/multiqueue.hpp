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
#include <any>
#include <condition_variable>
#include <deque>
#include <mutex>

class MultiQueue {
  public:
  MultiQueue(size_t size = 5000000);
  MultiQueue(const MultiQueue& other) = default;
  MultiQueue(MultiQueue&& other) = default;
  virtual ~MultiQueue() noexcept = default;
  MultiQueue& operator=(const MultiQueue& other) = default;
  MultiQueue& operator=(MultiQueue&& other) = default;

  void send(const std::any&);
  void receive(std::any&);
  bool try_receive(std::any&);

  protected:
  private:
  std::mutex key;
  std::condition_variable_any non_empty;
  std::condition_variable_any non_full;
  std::deque<std::any> fifo;
  size_t size;
};
