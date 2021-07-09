import networkx as nx
import matplotlib.pyplot as plt
import graphGenerator
import time

"""
The algorithm starts by picking a starting vertex and adding it to hpath(hamiltonian path)

Then recursively:
1. Gets all the neighbours of the last vertex in hpath and checks which of them are articulation points (ignores neighbours that already are in hpath)
2. Checks all the neighbours that aren't articulation points (and not in hpath) on wether they create any multi-articulation points(articulation points that creates 3 or more disconnected graphs when removed)
3. If it doesn't create multi-Articulation Points then it is added to hpath
and everything is done again recursively

"""

# test graphs that has a hamiltonian path
G1 = {0: [4, 1, 6, 3], 1: [0, 8, 9, 4, 7], 2: [7, 5, 4, 8], 3: [7, 9, 0], 4: [2, 1, 6, 0], 5: [2], 6: [9, 0, 4], 7: [3, 2, 1], 8: [1, 2], 9: [3, 6, 1]}
G2 = {0: [3, 9], 1: [7], 2: [7, 6, 3], 3: [7, 0, 8, 2], 4: [7, 9, 5], 5: [4, 6], 6: [2, 5], 7: [2, 3, 1, 4], 8: [3], 9: [4, 0]}
G3 = {0: [5, 1, 8, 3, 2], 1: [0, 6, 9], 2: [5, 7, 4, 0], 3: [8, 0], 4: [5, 9, 2], 5: [0, 4, 2], 6: [1, 7], 7: [2, 6], 8: [0, 3], 9: [4, 1]}

# test graphs with no hamiltonian path (UG = Unsolvable Graph)
UG1 = {0: [5], 1: [5, 6], 2: [5, 3, 7, 8], 3: [2, 5], 4: [7], 5: [2, 7, 1, 0, 3], 6: [1], 7: [5, 4, 8, 2], 8: [7, 9, 2], 9: [8]}
UG2 = {0: [3, 7, 6], 1: [6, 5, 8], 2: [8, 4], 3: [0, 5], 4: [2, 5], 5: [3, 8, 9, 4, 1], 6: [0, 1], 7: [0], 8: [5, 2, 1], 9: [5]}
UG3 = {0: [8, 7], 1: [9, 3], 2: [5], 3: [5, 1], 4: [9, 6], 5: [2, 8, 3, 6], 6: [4, 5], 7: [8, 0], 8: [5, 9, 7, 0], 9: [8, 4, 1]}
UG4 = {0: [17, 11, 8, 2], 1: [19, 6, 3], 2: [15, 24, 7, 16, 6, 0], 3: [1, 16], 4: [17, 26], 5: [28, 13, 17, 21], 6: [1, 19, 2], 7: [28, 22, 2], 8: [10, 13, 15, 0], 9: [29, 27], 10: [17, 8], 11: [19, 0], 12: [17], 13: [5, 8, 27, 22, 24], 14: [18, 19, 29, 22], 15: [19, 2, 27, 20, 8], 16: [18, 25, 2, 27, 3, 28], 17: [19, 4, 10, 0, 12, 5, 26, 28], 18: [14, 16, 26, 22, 25], 19: [14, 15, 17, 11, 1, 6], 20: [15], 21: [23, 5], 22: [14, 18, 7, 23, 13], 23: [29, 21, 22], 24: [28, 2, 13], 25: [16, 29, 18], 26: [18, 28, 4, 17], 27: [15, 16, 9, 13], 28: [26, 7, 24, 5, 16, 17], 29: [14, 23, 9, 25]}

# checks wether there is a hamiltonian path starting from u
def getHamiltonianPath(u, g):
    hpath = [] # hamiltonian path

    hpath.append(u) # hamiltonian path starts from u

    hpath = findHamiltonianPath(g, hpath)

    return hpath

def findHamiltonianPathForAllStartingVertices(g):
    hpath = []
    for i in range(len(g)):
        hpath = getHamiltonianPath(i, g)
        if(hpath != []):
            break
    return hpath

def createsMultiArticulationPoint(g, u, hpath): # true if the articulation point creates 3 or more disconnected graphs when removed
    # finds the articulation points after u is removed from the graph
    hpathCopy = list(hpath)
    hpathCopy.append(u)
    ap = GetArticulationPointsDegrees(hpathCopy, g)

    # checks whether any of the articulation points in ap are multi-articulation points
    for i in range(len(ap)):
        if(ap[i] > 1): # true if the articulation point creates 3 or more disconnected graphs when removed
            return True 
    return False

def ArticulationPointSearch(g, u, timeSteps, ap, visited, lowTime, discoveryTime, parent):

    timeSteps += 1
    visited[u] = True

    # initialize the current vertex
    children = 0
    lowTime[u] = timeSteps
    discoveryTime[u] = timeSteps

    # check all the unvisited neigbours
    for v in g[u]:
        if not visited[v]:
            if parent[u] == -1: children += 1 # counts the children of the root
            parent[v] = u
            ArticulationPointSearch(g, v, timeSteps, ap, visited, lowTime, discoveryTime, parent)

            # if u is the root of the DFS tree and it has two or more children it is an articulation point
            if parent[u] == -1 and children > 1:
                ap[u] += 1

            # if a child has a lowTime >= to the discovery time of u, u is an articulation point (except if u is the root)
            if parent[u] != -1 and lowTime[v] >= discoveryTime[u]:
                ap[u] += 1

            # see if u has a child with a back edge, and update the lowTime of u
            lowTime[u] = min(lowTime[u], lowTime[v])

        # check if its a back edge
        elif v != parent[u]:
            # see if the back edge of u has a smaller lowTime than the discovery time of v
            lowTime[u] = min(lowTime[u], discoveryTime[v])


# finds articulation points and how many disconnected graphs the articulation point creates when removed using DFS-traversal
def GetArticulationPointsDegrees(hpath, g):
    timeSteps = 0
    vertexCount = len(g)

    ap = [0] * vertexCount # Stores how many disconnected graphs that is created when the vertex is removed (for each vertex)
    # All the vertices  that aren't already in hpath starts off as unvisited
    visited = [False] * vertexCount
    for i in hpath:
        visited[i] = True

    lowTime = [float("Inf")] * vertexCount
    discoveryTime = [float("Inf")] * vertexCount
    parent = [-1] * vertexCount

    # start the ArticulationPointSearch from a vertex not visited (u becomes the root of the DFS tree)
    for u in range(vertexCount):
        if not visited[u]:
            ArticulationPointSearch(g, u, timeSteps, ap, visited, lowTime, discoveryTime, parent)
    return(ap)

def findHamiltonianPath(g, hpath):
    neighbours = []
    for neighbour in g[hpath[-1]]:
        if(neighbour not in hpath): neighbours.append(neighbour)

    ap = GetArticulationPointsDegrees(hpath, g) # finds articulation points and how many disconnected graphs the articulation point creates when removed

    finishedPath = []

    for i in range(len(neighbours)):
        if(ap[neighbours[i]] < 1): # if the neighbour is not an articulation point
            if(not createsMultiArticulationPoint(g, neighbours[i], hpath)):
                newHpath = list(hpath)
                newHpath.append(neighbours[i])
                finishedPath = findHamiltonianPath(g, newHpath)

                if(len(finishedPath) == len(g)):
                    return finishedPath
                elif(finishedPath != []):
                    return []
    if(len(neighbours) == 0):
        return hpath
    if(len(finishedPath) > 0):
        return finishedPath
    else:
        return []

def visualizeGraph(g, hpath):
    G = nx.Graph()
    # add the edges (and vertices) of the graph
    for i in range(len(g)):
        for j in range(len(g[i])):
            G.add_edge(i, g[i][j], color='k')

    # color the edges in the hpath red
    for i in range(len(hpath)-1):
        G.add_edge(hpath[i], hpath[i+1], color='r')

    edges = G.edges()
    colors = [G[u][v]['color'] for u,v in edges]

    nx.draw_networkx(G, edge_color=colors)
    plt.show()


vertices = 10
edges = 14
# generates a random connected graph with generateGraph(amount of vertices, amount of edges) (the amount of edges must be equal or greater than the amount of vertices - 1)
graph = graphGenerator.generateGraph(vertices, edges)
#print("The graph as an adjecency list: " + str(graph))

start_time = time.time()
hamiltonianPath = findHamiltonianPathForAllStartingVertices(graph)
if(hamiltonianPath == []):
    print("There is no hamiltonian path")
else:
    print("Hamiltonian Path was found: " + str(hamiltonianPath))

print("--- %s seconds ---" % (time.time() - start_time)) # prints the time it took to find the hamiltonian path (or time it took to find that there is no hamiltonian path)

visualizeGraph(graph, hamiltonianPath)