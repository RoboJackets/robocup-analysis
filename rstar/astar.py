import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop
import time
import cProfile
from node import Node
from stateGrid import Obstacles

# program should take as param
# start, end, statetrasnsition function and heuristic function

openList = [] #Contains Nodes
closedList = set([]) #Contains Nodes
#obstacles = []
#obstacles = [(3, 1), (3, 2), (3, 3), (3, 4), (3, 5), (3, 6)] # Might need to be vectors?
#obstacles = [(0, 1), (1, 1), (2, 1), (3, 1), (5, 1), (1, 3), (2, 3), (3, 3), (4, 3), (5, 2), (6, 5), (6, 6), (6, 7)] # Might need to be vectors?
totalTime1 = 0
timeTests = []

def timefunc(f): # used to time main function
    def f_timer(*args, **kwargs):
        global timeTests
        start = time.time()
        result = f(*args, **kwargs)
        end = time.time()
        timeTests += [end-start]
        print(f.__name__, 'took', end - start, 'time')
        return result
    return f_timer


#@timefunc
def astar(start, end, heuristicFunc, stateTransFunc, obstacles, costFunc = None): #start, end of ANY type.  hF and STF can reutnr ANY type
    if costFunc == None:
        costFunc = heuristicFunc
    startT = time.time()
    origin = Node(start, np.array([[]]), 0, heuristicFunc(start, end))
    heappush(openList, origin)

    searching = True
    while(searching):
        currentNode = heappop(openList)
        #print(currentNode.state, currentNode.g, currentNode.h)
        closedList.add(currentNode)
        addPossToOpenList(currentNode, end, heuristicFunc, stateTransFunc, obstacles, costFunc)

        #print(("Current: %s Target: %s Dist: %d" %(currentNode.state, end, np.linalg.norm(currentNode.state - end))))
        #print("__________________________________________")

        astarDelta = 6
        if np.linalg.norm(currentNode.state[:2, :] - end[:2, :]) < 15: # Target Found
            searching = False
            endT = time.time()
            elap = endT - startT
            path = traceBack(currentNode, origin)
            if elap > 0.4:
                print("start: %s   end: %s   elap: %.2f   expansions: %d" %(start, end, elap, len(path)))
                return None
            #else:
            #   print("Not extra long: %.6f  expansions: %d" %(elap, len(path)))

            #print("________________________________________________________")
            #plotPaths(start, end, path, obstacles)
            #print(len(path))
            return path

def addPossToOpenList(sourceNode, end, heuristicFunc, stateTransFunc, obstacles, costFunc): # Slowest Function by far
    global totalTime1
    possArr = stateTransFunc(sourceNode.state, obstacles)

    for poss in possArr:
        g = sourceNode.g + costFunc(poss, sourceNode.state)
        h = heuristicFunc(poss, end)

        possNode = Node(poss, sourceNode, g, h)


        # if possNode not in closedList:
        #     for op in openList: # NOTE: there should be a way not to have to check entire openList
        #         if possNode == op: # If node exists in openList this determines if it needs to be re-parented
        #             if op.g > heuristicFunc(op.state, sourceNode.state) + sourceNode.g:
        #                 #print("got here")
        #                 op.parent = sourceNode
        #                 op.g = heuristicFunc(op.state, sourceNode.state) + sourceNode.g
        #                 op.h = heuristicFunc(op.state, end)
        #             break
        heappush(openList, possNode)
        #totalTime1 += (endT - startT)


def traceBack(end, start):
    path = [end]
    #print("Reached the end Node!!")
    atOrigin = end == start

    while (not atOrigin):
        end = end.parent
        path = [end] + path
        atOrigin = end == start

    return path

def plotPaths(start, end, path, obstacles):
    xVals = [x.state[0][0] for x in path] # x/y of path
    yVals = [y.state[1][0] for y in path]

    xObsVals = [x[0] for x in obstacles.grid] # x/y of obstacles
    yObsVals = [y[1] for y in obstacles.grid]

    plt.plot(xVals,yVals)
    plt.plot(xObsVals, yObsVals, 'x', color = "red")
    plt.plot(xVals,yVals,'or')
    plt.plot(start[0][0], start[1][0], 'o', color = "blue")
    plt.plot(end[0][0], end[1][0], 's', color = "green")

    dx = abs(start[0][0] - end[0][0])
    dy = abs(start[1][0] - end[1][0])
    print("dx: %d  |  dy: %d" %(dx, dy))
    dim = dx if (dx >= dy) else dy

    plt.axis([-1, dim + 1, -1, dim + 1])
    plt.show()

def printHeap(heap):
    for i in heap:
        print(i)

def testTimeComplex():
    j = 1
    xVals = []
    for i in range (1, 6):
        xVals += [i]
        astar(np.array([[0, 0]]).T, np.array([[j, j]]).T, heuristic, stateTransition, None)
        j *= 2


    plt.plot(xVals,timeTests)
    plt.plot(xVals, timeTests, 'or')
    plt.axis([-1, 6, -1, 6])
    plt.show()

def heuristic(start, end): #start end can be anything.  make them vectors for no
    dx = start[0][0] - end[0][0]
    dy = start[1][0] - end[1][0]
    #return (abs(dx) + abs(dy))**0.5s
    return (abs(dx) + abs(dy))

def stateTransition2(start, obstacles): #Returns list of any hashable element
    x = start[0][0]
    y = start[1][0]

    possArr = []
    #for delta in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, -1), (-1, 1)): # Allowed set of dx and dy
    for delta in ((5, 0), (0, 5), (0, -5), (5, 5), (5, -5), (-5, -5), (-5, 5)): # Allowed set of dx and dy
    #for delta in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        possX = x + delta[0]
        possY = y + delta[1]

        # "possX >= 0 and possY >= 0 and" 
        if (possX, possY) not in obstacles: # Assures path stays in bounds and away from obstacles
            possArr += [np.array([[possX, possY]]).T]

    return possArr


def stateTransition(start, obstacles): #Returns list of any hashable element
    possArr = []
    #for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, -1), (-1, 1)]: # Allowed set of dx and dy
    for (dx, dy) in ((5, 0), (0, 5), (0, -5), (5, 5), (5, -5), (-5, -5), (-5, 5)):
        pos = start + np.array([[dx], [dy]])
        if not obstacles.isCollision(pos):
            possArr += [pos]

    return possArr


def main():
    obstacles = [(3, 1), (3, 2), (3, 3), (3, 4), (3, 5), (3, 6)] # Might need to be vectors?
    obstacleGrid = Obstacles(obstacles)
    astar(np.array([[0, 0]]).T, np.array([[100, 100]]).T, heuristic, stateTransition, obstacleGrid)

#main()