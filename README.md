# AStarOnGrid
A* Algorithm to find the shortest path between two nodes on a grid with walls.

Approach: A Star (A*) algorithm with some custom changes. On a very small grid (like in the examples)
the Dijkstra Algorithm for path finding is probably faster than the simplest version of A*,
because A* has to calculate its node elapsing heuristic with every expanding step,
but since Dijkstra spreads uninformed into all directions A* is on bigger maps faster.
