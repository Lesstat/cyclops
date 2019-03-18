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

struct SimilarityPolicy {
  double compare(size_t, size_t) { return 0; };
  bool similar(size_t, size_t) { return false; };
};

template <int Dim, class Derived> struct SharingSimilarityPolicy : public SimilarityPolicy {
  double compare(size_t i, size_t j)
  {
    Derived* derived = static_cast<Derived*>(this);
    if (i > j) {
      std::swap(i, j);
    }
    std::pair index = { i, j };
    if (similarities.count(index) > 0) {
      return similarities[index];
    }
    auto similarity = calculateSharing(derived->route(i), derived->route(j));
    similarities[index] = similarity;
    return similarity;
  }
  bool similar(size_t i, size_t j) { return compare(i, j) > max_overlap; }

  void set_overlap(double overlap) { max_overlap = overlap; }

  private:
  std::map<std::pair<size_t, size_t>, double> similarities;
  double max_overlap = 1;
};
