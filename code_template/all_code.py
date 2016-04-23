# Note that infinity can be represented by float('inf') in Python.

################################################################################
# You do not need to implement anything in this section below.

def dist(loc1, loc2):
    xdiff = loc1[0] - loc2[0]
    ydiff = loc1[1] - loc2[1]
    return math.sqrt(xdiff * xdiff + ydiff * ydiff)

import heapq
import itertools
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

# TODO: Implement part (c).
def bidirectional(n, edges, source, target):
    pass

# TODO: Implement part (d).
def astar(locs, edges, source, target):
    pass
