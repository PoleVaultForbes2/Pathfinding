# Pathfinder (A* Pathfinding Algorithm)
This Java class implements the A* (A-Star) pathfinding algorithm to find a near-optimal path between two points on a 2D grid terrain. It is designed to work with a provided visualizer that displays the computed path.

**How It Works**

The Pathfinder class uses a modified A* search algorithm to efficiently navigate a grid, accounting for variable terrain costs and 8-directional movement (horizontal, vertical, and diagonal). It relies on:
* **`PFNode`**: Internal node class that tracks position **`(Coord)`**, cost from the start, and a link to the previous node.
* **`MinPQ`**: A minimum priority queue that orders nodes based on total estimated cost:
**`f(n) = g(n) + h(n)`**
where **`g(n)`** is the travel cost from the start, and **`h(n)`** is the heuristic estimate to the goal.
* **`NodeState[][]`**: A 2D array that tracks whether each cell is unvisited, open (in the queue), or closed (processed).

**Key Features**
* Diagonal Movement: The algorithm considers up to 8 neighbors per node for more flexible pathfinding.
* Terrain Cost Awareness: Actual travel costs between grid locations are determined by the Terrain class.
* Heuristic Scaling: A user-defined heuristic multiplier allows tuning between greedy search and uniform-cost search.
* Efficient Node Management: Avoids re-processing of closed nodes, and re-evaluates open nodes when a cheaper path is found.
* Path Reconstruction: Backtracks from the end node to the start to build the final path as a list of Coord objects.

**Public Methods Overview**
* **`setPathStart(Coord)`**, **`setPathEnd(Coord)`**: Define the start and goal positions.
* **`setHeuristic(float):`** Set the weight of the heuristic function.
* **`computePath():`** Runs the A* algorithm from the start to the goal.
* **`getPathSolution():`** Returns the reconstructed path as an **`Iterable<Coord>`**.
* **`getPathCost():`** Returns the total cost of the path found.
* **`getSearchSize():`** Returns the number of nodes expanded during the search.
* **`wasSearched(Coord):`** Returns whether a given location was examined.

**Running the Pathfinder**

To run the pathfinding algorithm and see it visualized:

Use the provided visualizer. This class is designed to integrate into the existing project framework, and does not include a main method. The visualizer initializes the terrain and coordinates, and displays the computed path based on the Pathfinder output.

Make sure to:
1.) Set both the start and end coordinates via setPathStart() and setPathEnd().

2.) Choose a heuristic weight using setHeuristic().

3.) Call computePath() to execute the A* search.

4.) Retrieve the path using getPathSolution() if a path was found.
