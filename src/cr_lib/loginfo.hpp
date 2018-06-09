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
#ifndef LOGINFO_H
#define LOGINFO_H

class Logger {

  Logger();

  static thread_local Logger instance;
  std::string info;

  public:
  static Logger* getInstance();

  void init();

  template <typename T> Logger& operator<<(const T& /*s*/)
  {
    // info += s;
    return *this;
  }

  Logger& operator<<(const double& /*d*/)
  {
    // info += std::to_string(d);
    return *this;
  }

  Logger& operator<<(const size_t& /*d*/)
  {
    // info += std::to_string(d);
    return *this;
  }

  std::string& getInfo();

  virtual ~Logger();

  static Logger* initLogger()
  {
    auto log = getInstance();
    log->init();
    return log;
  }
};

#endif /* LOGINFO_H */
