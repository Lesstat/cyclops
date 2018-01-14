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

void MultiQueue::send(const std::any& value)
{
  std::lock_guard guard(key);
  fifo.push_back(value);
  non_empty.notify_all();
}

void MultiQueue::receive(std::any& value)
{
  std::lock_guard guard(key);
  non_empty.wait(key, [this] { return !fifo.empty(); });
  value = fifo.front();
  fifo.pop_front();
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
