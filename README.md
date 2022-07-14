# astar_search_maze
Create an astar search algorithm that can find the optimal solution to a maze with one or many waypoints. 

### Minimum Spanning Tree
Instead of computing the distances to each waypoint from the current position, it would be more helpful to obtain an estimate of the cost of reaching the rest of the unreached waypoints once we have reached one. 

Obtaining this estimate can be done with an MST: by constructing a graph where the vertices are the waypoints and each edge connecting w_i to w_j has weight manhattan_distance(w_i, w_j) for all pairs of vertices (w_i, w_j), the MST represents the approximate lowest cost path that connects all the waypoints. Since it strictly underestimates the cost of going through all the waypoints, this is an admissable heuristic.

Requirements: 

`pip3 install pygame`

To Run:

Replace 'small' with 'tiny' for a different, less exciting maze:

`python3 main.py data/part-2/small --search astar_single`

`python3 main.py data/part-3/small --search astar_multiple`

Run 

`python3 main.py --human data/part-1/small`

To play it yourself :).
