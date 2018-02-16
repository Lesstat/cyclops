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
#ifndef SERIALIZE_OPTIONAL_H
#define SERIALIZE_OPTIONAL_H

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost {
namespace serialization {

  template <class Archive, class T>
  void save(Archive& ar, const std::optional<T>& t, const unsigned int /*version*/
  )
  {
    const bool tflag = t.has_value();
    ar << boost::serialization::make_nvp("initialized", tflag);
    if (tflag) {
      ar << boost::serialization::make_nvp("value", *t);
    }
  }

  template <class Archive, class T>
  void load(Archive& ar, std::optional<T>& t, const unsigned int /*version*/
  )
  {
    bool tflag;
    ar >> boost::serialization::make_nvp("initialized", tflag);
    if (!tflag) {
      t.reset();
      return;
    }

    if (!t.has_value()) {
      t = T();
    }
    ar >> boost::serialization::make_nvp("value", *t);
  }

  template <class Archive, class T>
  void serialize(Archive& ar, std::optional<T>& t, const unsigned int version)
  {
    boost::serialization::split_free(ar, t, version);
  }
}
}

#endif /* SERIALIZE_OPTIONAL_H */
