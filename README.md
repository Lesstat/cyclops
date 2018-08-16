# Cyclops - an alternative route planning tool for bicycles

## Building
To Build Cyclops first clone it and change into its directory:

``` shell
git clone --recursive https://github.com/lesstat/cycle-routing
cd cycle-routing
```

Then create a build directory for cmake and run cmake and make.


``` shell
mkdir build
cd build
cmake ..
make -j
```

## Usage
The main executable of Cyclops is ``cr``. It has the following cli options:
``` shell
$ ./build/cr -h
  -h [ --help ]          prints help message

loading options:
  -t [ --text ] arg      load graph from text file
  -b [ --bin ] arg       load graph form binary file
  -m [ --multi ] arg     load graph from multiple files

actions:
  --save arg             save graph to binary file
  --test                 runs normal dijkstra and CH dijktra for comparison
  -w [ --web ]           start webserver for interaction via browser
```
It needs exactly one of the loading category to load a graph and at
least one of the second category. The actions are executed in the
order they are displayed above. 

``--save`` prompts the application
write the graph to a binary file for faster loading (no text parsing necessary).

``--test`` executes 1,000 random source-target queries with Dijkstra
and CH-Dijkstra, assures that they output the same path and prints the
speed-up at the end.

``--web`` starts a web server for interactive use.

## Graph Data
Cyclops can read road graphs in two different formats. The important
one is a text format. The other one is a binary format which Cyclops
can write from any valid input graph and read again. It is used for
faster reading and writing of graphs which should not be modified any
more.

The text format consists of a header and a body.

### Header
The header starts with an arbitrary number of lines reserved for
comments. These lines start with and ``#``. They are followed by a
blank line and afterwards three lines defining the contents of the
file:
- Dimension of the cost vector
- Number of nodes
- Number of edges

A header could look like this:
``` 
# Creation date: 02.03.2018
# Edge Costs: length, travel time, positive height ascent

3
8456123
17987123
```
Which means there is a vector of size three assigned to every
edge. There are 8,456,123 nodes in the graph and 17,987,123 edges.

### Nodes
Immediately after the header (without a blank line) follows the data for all nodes, one node
per line:
``` 
0 125799 53.0749415 8.7868047 3.4728513520111997 0
``` 
Those values are:
```
<internal id> <osmid> <latitude> <longitude> <height> <CH-Level>
```
No two nodes are allowed to have the same internal id. The CH-Level is
important for the speeding up the shortest path search. A Ch-Graph can
for example be constructed from a normal graph by my [multi-ch-creator](https://github.com/lesstat/multi-ch-creator).
Cyclops works with a normal graph too, but slowly :-)

### Edges
After the last node the edges of the graph start. An edge looks like
this:
``` 
22 104289 6 0.17316000000255372 1.5 -1 -1
```
which stands for:
``` 
<internal source node id> <internal destinantion node id> <cost 1>
<cost 2> .. <cost dimension> <replaced edge id 1> <replaced edge id 2>
```
Edges are always one way. The replaced edge ids are for shortcut edges in
a CH-Graph. Normal edges don't replace other edges and there for have
``-1`` as replaced edge id. An edge is assigned an internal id by the
placement in the file. The first edge has id 0, the second id 1, and
so on.





