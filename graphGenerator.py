import random
import math  

# pseudo code:
# parameters: amount of vertices V and amount of edges E
# create V amount of vertices from 0 to V-1
# make the graph connected:
#   make a random vertex connected to another random vertex
#   make the next random vertex with 0 neighbours connected to one of the vertices that already has at least 1 neighbour
#   repeat for all vertices
# randomly create the remaining (E-V) edges
#   Pick a random vertex that isn't fully connected
#   generate a list of vertices that the vertex isn't connected to and pick a vertex from that list that isn't fully connected
#   

def generateGraph(V, E):

    if(not E >= V - 1):
        raise Exception("The amount of edges must be equal to or greater than the amount of vertices - 1 ")
    if(E > (V-1)*((V-1)+1)/2):
        raise Exception("The amount of edges cant be larger than (V-1)*((V-1)+1)/2 where V is the amount of vertices")

    adj = {}
    # generates the vertices
    for i in range(V):
        adj[i] = []

    unconnected = list(adj)
    connected = []

    # make a random vertex connected to another random vertex
    vertex = unconnected[math.floor(random.random()*len(unconnected))]
    unconnected.pop(unconnected.index(vertex))
    newNeighbour = unconnected[math.floor(random.random()*len(unconnected))]
    unconnected.pop(unconnected.index(newNeighbour))
    adj[vertex].append(newNeighbour)
    adj[newNeighbour].append(vertex)
    connected.append(vertex)
    connected.append(newNeighbour)

    #make the remaining random vertices with 0 neighbours, connected to one of the vertices that already has at least 1 neighbour
    for i in range(V-2):
        vertex = unconnected[math.floor(random.random()*len(unconnected))]
        unconnected.pop(unconnected.index(vertex))
        newNeighbour = connected[math.floor(random.random()*len(connected))]
        adj[vertex].append(newNeighbour)
        adj[newNeighbour].append(vertex)
        connected.append(vertex)

    if(E-V+1 > 0):
        
        # creates a list of vertices that aren't fully connected
        verticesNFC = list(adj) # a list of all vertices that are Not Fully Connected, meaning they aren't neighbours with all other vertices
        for i in range(len(verticesNFC)):
            if(len(adj[i]) == len(adj)-1):
                verticesNFC.pop(verticesNFC.index(i))

        # randomly creates the remaining (E-V) edges
        for i in range(E-V+1):
            vertex = verticesNFC[math.floor(random.random()*len(verticesNFC))]
            notNeighbours = list(adj)
            notNeighbours.pop(notNeighbours.index(vertex))
            for j in range(len(adj[vertex])):
                notNeighbours.pop(notNeighbours.index(adj[vertex][j]))
            newNeighbour = notNeighbours[math.floor(random.random()*len(notNeighbours))]
            adj[vertex].append(newNeighbour)
            adj[newNeighbour].append(vertex)
            if(len(adj[vertex]) == len(adj)-1):
                verticesNFC.pop(verticesNFC.index(vertex))
            if(len(adj[newNeighbour]) == len(adj)-1):
                verticesNFC.pop(verticesNFC.index(newNeighbour))

    return adj