# My Hamiltonian Path Algorithm

The algorithm takes any undirected connected graph (represented as an adjacency list with vertices named from 0 to the amount of vertices - 1, e.g. {0: [3, 2], 1: [3, 2], 2: [3, 1, 0], 3: [1, 0, 2]}) and prints the first hamiltonian path it found (and returns it as a list), or if there is no hamiltonian path in the graph it prints "There is no hamiltonian path" (and returns an empty list). All the important code for this project is in the hamiltonianPathAlgorithm.py file. 

The logic behind the algorithm works like this:

Start by picking a starting vertex and adding it to hpath(hamiltonian path)

Then recursively:
1. Gets all the neighbours of the last vertex in hpath and checks which of them are articulation points (ignores neighbours that already are in hpath)
2. Checks all the neighbours that aren't articulation points (and not in hpath) on whether they create any multi-articulation points(articulation points that creates 3 or more disconnected graphs when removed)
3. If it doesn't create multi-Articulation Points then it is added to hpath
and everything is done again recursively

This logic seems to work on all undirected connected graphs, and significantly speeds up the process of finding a hamiltonian path as compared to just trying all possible paths
