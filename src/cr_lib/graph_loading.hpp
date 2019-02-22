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
#include "graph.hpp"
#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <chrono>
#include <fstream>
#include <iostream>

using ms = std::chrono::milliseconds;
namespace iostr = boost::iostreams;

template <int Dim> Graph<Dim> loadGraphFromTextFile(std::string& graphPath, bool zipped)
{
  using Graph = Graph<Dim>;
  const size_t N = 256 * 1024;
  char buffer[N];
  std::ifstream graphFile {};
  graphFile.rdbuf()->pubsetbuf((char*)buffer, N);
  graphFile.open(graphPath);

  iostr::filtering_istream in;
  if (zipped)
    in.push(iostr::gzip_decompressor());
  in.push(graphFile);

  std::cout << "Reading Graphdata" << '\n';
  auto start = std::chrono::high_resolution_clock::now();
  Graph g = Graph::createFromStream(in);
  auto end = std::chrono::high_resolution_clock::now();

  std::cout << "creating the graph took " << std::chrono::duration_cast<ms>(end - start).count()
            << "ms" << '\n';
  return g;
}

template <int Dim> Graph<Dim> loadGraphFromBinaryFile(std::string& graphPath)
{
  using Graph = Graph<Dim>;
  std::ifstream binFile { graphPath };
  boost::archive::binary_iarchive bin { binFile };

  std::cout << "Reading Graphdata" << '\n';
  auto start = std::chrono::high_resolution_clock::now();
  Graph g = Graph::createFromBinaryFile(bin);
  auto end = std::chrono::high_resolution_clock::now();

  std::cout << "creating the graph took " << std::chrono::duration_cast<ms>(end - start).count()
            << "ms" << '\n';
  return g;
}
