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
#include "multiqueue.hpp"

MultiQueue::MultiQueue(size_t size)
    : size(size)
{
}

void MultiQueue::send(const std::any& value)
{
  std::unique_lock guard(key);
  non_full.wait(guard, [this] { return size > fifo.size(); });
  fifo.push_back(value);
  non_empty.notify_one();
}

std::any MultiQueue::receive()
{
  std::unique_lock guard(key);
  non_empty.wait(guard, [this] { return !fifo.empty() || closed_; });
  if (fifo.empty() && closed_) {
    return std::any();
  }
  auto value = fifo.front();
  fifo.pop_front();
  non_full.notify_one();
  return value;
}

bool MultiQueue::try_receive(std::any& value)
{
  std::lock_guard guard(key);
  if (fifo.empty()) {
    return false;
  }
  value = fifo.front();
  fifo.pop_front();
  return true;
}

size_t MultiQueue::receive_some(std::vector<std::any>& container, size_t some)
{
  std::unique_lock<std::mutex> guard(key);
  non_empty.wait(guard, [this] { return !fifo.empty() || closed_; });
  while (!fifo.empty() && container.size() < some) {
    container.push_back(fifo.front());
    fifo.pop_front();
  }
  return container.size();
}

void MultiQueue::close()
{
  std::lock_guard guard(key);
  closed_ = true;
}

bool MultiQueue::closed()
{
  std::lock_guard guard(key);
  return closed_;
}
