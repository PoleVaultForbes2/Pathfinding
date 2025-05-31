# Pathfinder (A* Pathfinding Algorithm)
This Java class implements the A* (A-Star) pathfinding algorithm to find a near-optimal path between two points on a 2D grid terrain. It is designed to work with a provided visualizer that displays the computed path.

**How It Works**

The Pathfinder class uses a modified A* search algorithm to efficiently navigate a grid, accounting for variable terrain costs and 8-directional movement (horizontal, vertical, and diagonal). It relies on:
* PFNode: Internal node class that tracks position (Coord), cost from the start, and a link to the previous node.
* MinPQ: A minimum priority queue that orders nodes based on total estimated cost:
f(n) = g(n) + h(n)
where g(n) is the travel cost from the start, and h(n) is the heuristic estimate to the goal.
* NodeState[][]: A 2D array that tracks whether each cell is unvisited, open (in the queue), or closed (processed).
