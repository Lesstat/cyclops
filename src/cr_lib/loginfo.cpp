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
#include "loginfo.hpp"

#include <algorithm>

Logger::Logger() {}

thread_local Logger Logger::instance;
std::string info;

Logger* Logger::getInstance() { return &instance; }
void Logger::init() { info.clear(); }

Logger::~Logger() {}

std::string& Logger::getInfo()
{
  info.erase(info.begin(),
      std::find_if(info.begin(), info.end(), [](int ch) { return !std::isspace(ch); }));
  info.erase(
      std::find_if(info.rbegin(), info.rend(), [](int ch) { return !std::isspace(ch); }).base(),
      info.end());
  return info;
}
