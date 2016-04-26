# Note that infinity can be represented by float('inf') in Python.

################################################################################
# You do not need to implement anything in this section below.

def dist(loc1, loc2):
    xdiff = loc1[0] - loc2[0]
    ydiff = loc1[1] - loc2[1]
    return math.sqrt(xdiff * xdiff + ydiff * ydiff)

import heapq
import itertools
import math
# Borrowed heavily from https://docs.python.org/2/library/heapq.html#priority-queue-implementation-notes
class PriorityQueue:
    def __init__(self):
        self.heap = []
        self.entry_finder = {}
        self.REMOVED = '<removed>'
        self.counter = itertools.count()
        self.num_elements = 0
        self.num_actions = 0

    def add(self, item, priority):
        if item in self.entry_finder:
            self.remove(item)
        count = next(self.counter)
        entry = [priority, count, item]
        self.entry_finder[item] = entry
        heapq.heappush(self.heap, entry)
        self.num_actions += 1
        self.num_elements += 1

    def remove(self, item):
        entry = self.entry_finder.pop(item)
        entry[-1] = self.REMOVED
        self.num_elements -= 1

    def pop(self):
        self.num_actions += 1
        while self.heap:
            priority, count, item = heapq.heappop(self.heap)
            if item is not self.REMOVED:
                self.num_elements -= 1
                del self.entry_finder[item]
                return item, priority
        raise KeyError('Pop from an empty priority queue')

    def head(self):
        priority, count, item = self.heap[0]
        return item, priority

    def empty(self):
        return self.num_elements == 0

# You do not need to implement anything in this section above.
################################################################################

# TODO: Implement both parts (a) and (b) with this function. If target is None,
# then return a list of tuples as described in part (a). If target is not None,
# then return a path as a list of states as described in part (b).
def dijkstra(n, edges, source, target=None):
    determinedVertices = set()
    priQueue = PriorityQueue()
    shortestLengthFromSource = {}
    parent = {}
    outputList = []
    shortestPath = []
    
    for vertex in edges.keys():
        if vertex != source:
            shortestLengthFromSource[vertex] = float("inf")
        else:
            shortestLengthFromSource[vertex] = 0
        priQueue.add(vertex, shortestLengthFromSource[vertex])    
        parent[vertex] = None
        
    #Double counting the source, but for now this is just a safety check
    #Once I debug through this, I'll determine whether it's needed or whether
    #I need to remove the if statement above    
    shortestLengthFromSource[source] = 0
    priQueue.add(source, 0)
    
    while not(priQueue.empty()):
        current = priQueue.pop()
        currentVertex = current[0]
        determinedVertices.add(currentVertex)
        
        if currentVertex == target:
            pathVertex = target
            while (pathVertex != None):
                shortestPath.append(pathVertex)
                pathVertex = parent[pathVertex]
                    
            shortestPath.reverse()
            return (shortestPath, shortestLengthFromSource[target])
        for vertexAndWeight in edges[currentVertex]:
            
            vertex = vertexAndWeight[0]
            weight = vertexAndWeight[1]
            if vertex not in determinedVertices:
                #Relax the edges
                if shortestLengthFromSource[vertex] > shortestLengthFromSource[currentVertex] + weight:
                    shortestLengthFromSource[vertex] = shortestLengthFromSource[currentVertex] + weight
                    priQueue.add(vertex, shortestLengthFromSource[vertex])
                    parent[vertex] = currentVertex
                                
        outputList.append((currentVertex, shortestLengthFromSource[currentVertex], parent[currentVertex]))
    return outputList

def stoppingCondition(u, v, bestPathWeight, shortestLengthForward, shortestLengthBackward,
                      weight, l_s, l_t, forwardParent, backwardParent, shortestPath, bestEdge):
    #Stopping condition
    edge = (u, v)
    possiblePathWeight = min(bestPathWeight, shortestLengthForward[u] + shortestLengthBackward[v] + weight)
    if possiblePathWeight < bestPathWeight:
        bestEdge = edge
        bestPathWeight = possiblePathWeight
    if (l_s + l_t >= bestPathWeight):
                
        currentForwardParent = bestEdge[0]
                
        while(currentForwardParent != None):
            shortestPath.append(currentForwardParent)
            currentForwardParent = forwardParent[currentForwardParent]
                
        #Since I was adding the parent of a node after I added the node itself, this list is in reverse order
        shortestPath.reverse()
                
        currentBackwardParent = bestEdge[1]
        #These nodes would be added in reverse order, but the order is already reversed, so the nodes are being added to 
        #the shortest path in the correct forward order
        while(currentBackwardParent != None):
            shortestPath.append(currentBackwardParent)
            currentBackwardParent = backwardParent[currentBackwardParent]
        return(shortestPath, bestPathWeight)
    return (False, bestPathWeight, bestEdge)
    
# TODO: Implement part (c).
def bidirectional(n, edges, source, target):
    processedForward = set() #S_s
    processedBackward = set() #S_t
    shortestLengthForward = {} #d_s
    shortestLengthBackward = {} #d_t
    bestPathWeight = float("inf") #mu
    bestEdge = (0,0)
    forwardPriQueue = PriorityQueue() #Q_s
    backwardPriQueue = PriorityQueue() #Q_t
    
    backwardEdges = {}
    
    #I need to essentially reverse all edges so that the backward dijkstra works (have the arrows pointing in the opposite
    #direction). This is where I populate backwardEdges as well as creating an easily searchable dictionary that gives
    #the forward edge weights between two nodes in the graph
    for vertex in edges.keys():        
        for tup in edges[vertex]:
            destVertex = tup[0]
            weight = tup[1]
            
            if destVertex not in backwardEdges.keys():
                backwardEdges[destVertex] = []
                
            backwardEdges[destVertex].append((vertex, weight))
            
    
    forwardParent = {} #pi_s
    backwardParent = {} #pi_t 
    
    shortestPath = []
    
    #Lowest path weight to unexpanded node from source. Basically, we check the forward queue and find
    #the minimum of it.
    l_s = 0
    #Lowest path weight to unexpanded node from target. Basically, we check the backward queue and find
    #the minimum of it
    l_t = 0

    forwardPriQueue.add(source, 0)
    backwardPriQueue.add(target, 0)
    shortestLengthBackward[target] = 0
    shortestLengthForward[source] = 0
    
    for vertex in range(n):
        shortestLengthForward[vertex] = float("inf")
        shortestLengthBackward[vertex] = float("inf")
        
        forwardPriQueue.add(vertex, shortestLengthForward[vertex]) 
        backwardPriQueue.add(vertex, shortestLengthBackward[vertex])   
        forwardParent[vertex] = None
        backwardParent[vertex] = None
    
    shortestLengthBackward[target] = 0
    backwardPriQueue.add(target, 0)
    shortestLengthForward[source] = 0
    forwardPriQueue.add(source, 0)
    
    
    while not(forwardPriQueue.empty()) or not(backwardPriQueue.empty()):
        currentForward = forwardPriQueue.pop()
        currentForwardVertex = currentForward[0]
        processedForward.add(currentForwardVertex)
        
        
        
        #Since both queues are min heaps, the head of the queues gives us the minimum weight of all
        #unexpanded nodes
        l_s = forwardPriQueue.head()[1]
        l_t = backwardPriQueue.head()[1]
        
#         if (currentForwardVertex in processedBackward) and (currentBackwardVertex in processedForward):
#             #Stopping condition
#             edge = (currentForwardVertex, currentBackwardVertex)
#             bestPathWeight = min(bestPathWeight, shortestLengthForward[currentForwardVertex] + shortestLengthBackward[currentBackwardVertex] + edgeWeights[edge])
#             if (l_s + l_t >= bestPathWeight):
#                 forwardParent[currentBackwardVertex] = currentForwardVertex
#                 
#                 
#                 currentForwardParent = currentForwardVertex
#                 
#                 while(currentForwardParent != None):
#                     shortestPath.append(currentForwardParent)
#                     currentForwardParent = forwardParent[currentForwardParent]
#                 
#                 #Since I was adding the parent of a node after I added the node itself, this list is in reverse order
#                 shortestPath = shortestPath.reverse()
#                 
#                 currentBackwardParent = currentBackwardVertex
#                 #These nodes would be added in reverse order, but the order is already reversed, so the nodes are being added to 
#                 #the shortest path in the correct forward order
#                 while(currentBackwardParent != None):
#                     shortestPath.append(currentBackwardParent)
#                     currentBackwardParent = backwardParent[currentBackwardParent]
# 
#             return(shortestPath, bestPathWeight)

        #Regular dijkstra stuff going both ways
        #Edges are directed though, so I might have a problem going backwards
        #Fixed the edges going backwards (theoretically)
        for vertexAndWeight in edges[currentForwardVertex]:
            
            vertex = vertexAndWeight[0]
            weight = vertexAndWeight[1]
            if vertex not in processedForward:
                #Relax the edges
                if vertex in processedBackward:
                    stopCondition = stoppingCondition(currentForwardVertex, vertex, bestPathWeight, shortestLengthForward,shortestLengthBackward,weight, l_s, l_t, forwardParent, backwardParent, shortestPath, bestEdge)
                    if (stopCondition[0] != False):
                        return stopCondition
                    else:
                        bestPathWeight = stopCondition[1]
                        bestEdge = stopCondition[2]
                if shortestLengthForward[vertex] > shortestLengthForward[currentForwardVertex] + weight:
                    shortestLengthForward[vertex] = shortestLengthForward[currentForwardVertex] + weight
                    forwardPriQueue.add(vertex, shortestLengthForward[vertex])
                    forwardParent[vertex] = currentForwardVertex
                    #Re-check the smallest weight in the forward queue
                    l_s = forwardPriQueue.head()[1]
        
        
        currentBackward = backwardPriQueue.pop()
        currentBackwardVertex = currentBackward[0]
        processedBackward.add(currentBackwardVertex)
        
        l_s = forwardPriQueue.head()[1]
        l_t = backwardPriQueue.head()[1]
                        
        for vertexAndWeight in backwardEdges[currentBackwardVertex]:
            
            vertex = vertexAndWeight[0]
            weight = vertexAndWeight[1]
            if vertex not in processedBackward:
                #Relax the edges
                if vertex in processedForward:
                    stopCondition = stoppingCondition(vertex, currentBackwardVertex, bestPathWeight, shortestLengthForward,shortestLengthBackward,weight, l_s, l_t, forwardParent, backwardParent, shortestPath, bestEdge)
                    if (stopCondition[0] != False):
                        return stopCondition
                    else:
                        bestPathWeight = stopCondition[1]
                        bestEdge = stopCondition[2]
                if shortestLengthBackward[vertex] > shortestLengthBackward[currentBackwardVertex] + weight:
                    shortestLengthBackward[vertex] = shortestLengthBackward[currentBackwardVertex] + weight
                    backwardPriQueue.add(vertex, shortestLengthBackward[vertex])
                    backwardParent[vertex] = currentBackwardVertex
                    #Re-check the smallest weight in the backward queue
                    l_t = backwardPriQueue.head()[1]
    
    return ([], float("inf"))
# TODO: Implement part (d).
def astar(locs, edges, source, target):
    determinedVertices = set()
    priQueue = PriorityQueue()
    shortestLengthFromSource = {}
    parent = {}
    outputList = []
    shortestPath = []
    
    for vertex in range(len(locs)):
        if vertex != source:
            shortestLengthFromSource[vertex] = float("inf")
        else:
            shortestLengthFromSource[vertex] = 0
        priQueue.add(vertex, shortestLengthFromSource[vertex])    
        parent[vertex] = None
        
    #Double counting the source, but for now this is just a safety check
    #Once I debug through this, I'll determine whether it's needed or whether
    #I need to remove the if statement above    
    shortestLengthFromSource[source] = 0
    priQueue.add(source, 0)
    
    locationTarget = locs[target]
    targetX = locationTarget[0]
    targetY = locationTarget[1]
    
    while not(priQueue.empty()):
        currentVertex = priQueue.pop()[0]
        locationCurrentVertex = locs[currentVertex]
        currentVertexX = locationCurrentVertex[0]
        currentVertexY = locationCurrentVertex[1]
        
        heurCurrent = math.sqrt((targetX - currentVertexX)**2 + (targetY - currentVertexY)**2)
        
        determinedVertices.add(currentVertex)
        
        if currentVertex == target:
            pathVertex = target
            while (pathVertex != None):
                shortestPath.append(pathVertex)
                pathVertex = parent[pathVertex]
                    
            shortestPath.reverse()
            return (shortestPath, shortestLengthFromSource[target])
        for vertex in edges[currentVertex]:
            locationVertex = locs[vertex]
            vertexX = locationVertex[0]
            vertexY = locationVertex[1]
            
            weight = math.sqrt((vertexX - currentVertexX)**2 + (vertexY-currentVertexY)**2)
            heurVertex = math.sqrt((vertexX-targetX)**2 + (vertexY - targetY)**2)
            
            if vertex not in determinedVertices:
                #Relax the edges
                if shortestLengthFromSource[vertex] > shortestLengthFromSource[currentVertex] + weight:
                    shortestLengthFromSource[vertex] = shortestLengthFromSource[currentVertex] + weight
                    priQueue.add(vertex, shortestLengthFromSource[vertex] + heurVertex - heurCurrent)
                    parent[vertex] = currentVertex
