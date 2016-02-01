## Week2 Code Design

### Author: Jan 31 2016, https://github.com/diegoo/ucsd_graphs_course

### Class: MapGraph

Modifications made to MapGraph (what and why): Since the data is sparse, I did not use the matrix implementation for the graph. First, I filled in the stubbed getters & setters, a couple of methods for debugging (showVertices() & showEdges()), and a constructor to set the vertices (a map of location -> vertices, as suggested in the lectures) and edges (a set, since there can't be repeated paths between vertices). The method addEdge() validates parameters before adding.

### Class name: MapEdge

Purpose and description of class: Encapsulate edge properties (beginning & end vertices, name, type, length). Since there are several parameters, I implemented a regular constructor and a static constructor using the Builder pattern, as suggested in the lectures.

### Class name: MapVertex

Purpose and description of class: Encapsulate vertex properties. A vertex 1) maps to one location on the map, and 2) has a set of edges radiating from it.

### Class name: Ancestors

Purpose and description of class: Encapsulate the result of the search (i.e. the path from start to goal). It does so by keeping updates to the path during search, so that steps can be traced back from goal to start when goal is reached. It uses a map of child -> parent: whenever a child is explored, its parent is stored. The method getPath(start, goal) begins pushing goal's parent into a stack, and then its parent's parent, all the way to the start -- then it pops from the stack to get the path from start to goal.

### Overall Design Justification:

The BFS is implemented with a queue (`agenda` in the code) keeping the vertices to visit, and a set of visited vertices (`visited` in the code). On each step of the loop, the current vertex is checked, marked as visited, and all its immediate neighbors added to the agenda for later visit. Also, the ancestors object is updated to keep track of the path to the current vertex. The search ends when the goal is found or the agenda is empty.
