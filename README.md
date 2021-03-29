# Instructions
  
  * Run the python script (evrp-script.py).
  * Follow the onscreen instructions (which have been provided below as well).
    * Sample test cases can be found in testcases.txt

# Input Formatting Guide

  **Please provide your input in the following format:**
  * All values provided in a single line must be separated by a single space.
  * First line should contain the number of cities (n), the number of connected roads (any one direction) and the number of electric vehicles (k).
  * Second line should contain the time period used to discretize time. 1s is a good place to start. Lesser values yield more accurate results, but tend to be more prone to     errors.
  * Follow this with n lines describing the roads, each containing the source, the destination, and the distance between, in order.
  * Lastly, enter k pairs of lines describing the vehicles, the first line of each containing the source and destination, in order, and the second, the initial battery charge, the charging rate, the discharging rate, the maximum battery capacity, and the average speed of the vehicle, in order.

# Heuristic Algorithm
  
  * A slightly-modified, but optimal Dijkstra's Algorithm (using min-heaps) is used to precalculate shortest paths between all pairs of nodes.
  * A heuristic underestimate of time is calculated, for each vehicle, as the sum of the time taken for the vehicle to traverse aforementioned shortest path at average speed, and the time taken to charge the vehicle just enough for it to be able to traverse said distance.

# Optimal Algorithm
  
  * The algorithm implements an optimized state-space search.
  * It takes into account, for each vehicle, the possibilites of:
    *  either charging at the present station (if it is not occupied, and if the vehicle does not have the least amount of charge required to reach the destination node from the present node via the shortest route between those two nodes, as given by the heuristic algorithm, without stopping for charging anywhere in between), or,
    *  waiting to charge at present station, or,
    *  moving on to any neighbouring node, provided that the vehicle has enough charge to move on to that node, and that the shortest path from that node to the destination of the vehicle does not pass through the current node (where the vehicle presently is).
  * After running passes through all the vehicles, the algorithm checks whether the present state is a valid solution, i.e., whether all vehicles have arrived.
    * If the present state is a valid solution, the algorithm compares the timestamp of the present state with the current solution (a global variable), and if the former is found to be lesser among the two, it is set as the new timestamp, and the present state is deepcopied to the solution state (also a global variable), after all of which, the algorithm goes back and continues the search.
    * If the present state is not a valid solution, the algorithm increments the state timestamp, and continues the search.
