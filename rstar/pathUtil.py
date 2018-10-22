import numpy as np
import math
import time
from heapq import heappush, heappop
from astar import astar
from node import Node
from stateGrid import Obstacles


def rstarTraceBack(end):
    sparsePath = [end]
    densePath = end.path
    #print("Reached the end Node!!")
    atOrigin = end.parent.state.size == 0

    while (not atOrigin):
        end = end.parent
        if end.path:
            densePath = end.path + densePath
        sparsePath = [end] + sparsePath
        atOrigin = (np.array_equal(end.parent, np.array([[]])))

    return (sparsePath, densePath)

class RStarCost:
    """
    params:
        avoid: True if this node is marked AVOID or False otherwise
        total_cost: the cost from the start to here plus the heuristic cost
            to the sparse goal state.
    """
    def __init__(self, avoid, total_cost):
        self.avoid = avoid
        self.total_cost = total_cost

    def __gt__(self, other):
        if self.avoid == other.avoid:
            return self.total_cost > other.total_cost
        else:
            return self.avoid

    def __ge__(self, other):
        if self.avoid == other.avoid:
            return self.total_cost >= other.total_cost
        else:
            return self.avoid

    def __lt__(self, other):
        if self.avoid == other.avoid:
            return self.total_cost < other.total_cost
        else:
            return other.avoid

    def __le__(self, other):
        if self.avoid == other.avoid:
            return self.total_cost <= other.total_cost
        else:
            return other.avoid

    def __eq__(self, other):
        return self.avoid == other.avoid and self.total_cost == other.total_cost

    def __str__(self):
        return "{{ {}, {} }}".format("AVOID" if self.avoid else "GOOD", self.total_cost)


class SparseNode(Node):
    """
    params:
        state: "sparse" state. Usually this will be a physical position in
            space (x, y).
        parent: the previous sparse node in the graph.
        g: the cost function from the start to this node.
        h: cached heuristic value from this node to the end.
        avoid: whether or not this node is marked as AVOID
    """
    def __init__(self, state, parent, g, h, avoid):
        Node.__init__(self, state, parent, g, h)
        self.path = None
        self.successors = []
        self.avoid = avoid

    @property
    def f(self):
        return RStarCost(self.avoid, self.g + self.h)

class RStarPlanner:
    def __init__(self, start, end, gammaTransition, sTransition, heuristic, w, obstacles, costFunc = None):
        self.astarTime = 0
        self.gammaTransition = gammaTransition
        self.sTransition = sTransition
        self.heuristic = heuristic
        self.w = w
        self.startState = start
        if costFunc is not None:
            self.costFunc = costFunc
        else:
            self.costFunc = heuristic

        self.obstacleGrid = Obstacles(obstacles)
        self.startNode = SparseNode(start, np.array([[]]), 0.0, self.w * self.heuristic(start, end), False) #start parent as empty array 
        self.endNode = SparseNode(end, None, math.inf, 0.0, True)
        self.openList = []
        self.closedList = set([])

        heappush(self.openList, self.startNode)

    def update_state(self, node):
        node.avoid = (node.g > self.w * self.heuristic(self.startNode.state, node.state) or (not node.path and node.avoid))
        node.h = self.w * self.heuristic(node.state, self.endNode.state)
        #node.g = node.parent.f.total_cost
        heappush(self.openList, node)


    def reevaluate(self, node):
        #print("STATE: %s, %s" % (node.parent.state, node.state))
        global astarTime
        start = time.time()
        initial_state = None
        if node.parent.path is not None:
            initial_state = node.parent.path[-1].state
        else:
            initial_state = self.startState
        path = astar(initial_state, node.state, self.heuristic, self.sTransition, self.obstacleGrid, self.costFunc) #TODO this needs to use the statespace not xy space
        end = time.time()
        self.astarTime += end - start

        if path is not None:
            node.c_low = path[-1].g
        else:
            node.c_low = self.heuristic(node.parent.state, node.state)

        node.path = path

        if node.path is None or node.parent.g + node.c_low > self.w * self.heuristic(self.startNode.state, node.state):
            # TODO Reevaluate node's parent
            node.avoid = True
        node.g = node.parent.g + node.c_low
        self.update_state(node)

    def plan(self):
        while self.openList and self.endNode >= self.openList[0] and self.endNode is not self.openList[0]: #changed to > only?
            minNode = heappop(self.openList)
            #print("CurrMin: %s  Goal: %s  State: (%s, %s)" %(minNode.f, self.endNode.f, minNode.state[0][0], minNode.state[1][0]))

            
            #print(minNode.state)
            if minNode is not self.startNode and not minNode.path:
                self.reevaluate(minNode)
            else:
                minNode.successors = self.gammaTransition(minNode, self.endNode, self.obstacleGrid)
                self.closedList.add(minNode)

                for sPrime in minNode.successors:
                    #print(sPrime)
                    #print("_______________")
                    sPrimeHeuristic = self.heuristic(sPrime, self.endNode.state)
                    sPrimeNode = SparseNode(sPrime, minNode, 0, sPrimeHeuristic, False)
                    sPrimeNode.c_low = self.heuristic(minNode.state, sPrime)
                    sPrimeNode.parent = None

                    if not sPrimeNode.parent or minNode.g + sPrimeNode.c_low < sPrimeNode.g:
                        sPrimeNode.g = minNode.g + sPrimeNode.c_low
                        sPrimeNode.h = self.heuristic(sPrimeNode.state, self.endNode.state)
                        sPrimeNode.parent = minNode
                        if (sPrimeNode.state[0][0] == self.endNode.state[0][0] and sPrimeNode.state[1][0] == self.endNode.state[1][0]):
                            self.endNode.avoid = False
                            self.endNode.h = 0
                            self.endNode.g = minNode.g
                            self.endNode.parent = minNode
                            self.endNode.path = astar(minNode.path[-1].state, self.endNode.state, self.heuristic, self.sTransition, self.obstacleGrid)
                            heappush(self.openList, self.endNode)
                        else:
                            self.update_state(sPrimeNode)
        paths = rstarTraceBack(self.endNode)
        print("A* Planning took %.2f seconds" %(self.astarTime))
        return(self.startNode.state, self.endNode.state, paths[0], paths[1], self.closedList)

