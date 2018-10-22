from astar import astar
import numpy as np
import numpy.linalg
import math
import random
import matplotlib.pyplot as plt 
from heapq import heappush, heappop
from pathUtil import RStarPlanner
from pathUtil import Node
from stateGrid import Obstacles, MovingObstacle
from animation import animate
import time

aMax = 300
vMax = 150

def gammaTransFunc(s, goal, obstacles):  #TODO make sure gamma trans checks collision
    #random.seed(20)
    #print(s.state)
    k = 30
    delta = 15
    succs = []

    x = s.state[0][0]
    y = s.state[1][0]
    goalx = goal.state[0][0]
    goaly = goal.state[1][0]

    for i in range(k):
        theta = random.randint(0, 360)
        theta = math.radians(theta)

        #directAngle = np.arctan2(goaly - y, goalx - x)
        #theta = np.random.normal(directAngle, .2)

        #print("direct: %.2f  theta:  %.2f" %(directAngle, theta))

        # 15.0 *
        valid = True
        #for sample in range (0, delta, 4):
        #    if obstacles.isCollision((x + sample*math.cos(theta), y + sample*math.sin(theta))):
        #        valid = False
        if valid:
            succs += [np.array([[x + delta*math.cos(theta)], [y + delta*math.sin(theta)]])]
    
    #directAngle = np.arctan2(goaly - y, goalx - x) 
    #succs += [np.array([[x + delta*math.cos(directAngle)], [y + delta*math.sin(directAngle)]])] 

    if math.hypot(x - goalx, y - goaly) < delta:
        print("IN RANGE OF GOAL.  SHOULD BE OVER?")
        succs += [np.array([[goalx], [goaly], [0], [0], [0]])]

    return succs

def cost(start, end):
    return end[4][0] - start[4][0]

def heuristic(start, end): #start end can be anything.  make them vectors for no
    return np.linalg.norm(start[:2, :] - end[:2, :]) / vMax

def diagonalHeuristic(start, end):
    x = start[0][0]
    y = start[1][0]
    goalx = end[0][0]
    goaly = end[1][0]

    D = 7
    D2 = 9.9

    dx = abs(x - goalx)
    dy = abs(y - goaly)
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

def stateTransition(start, obstacles): #Returns list of any hashable element
    possArr = []
    invl = 6
    for (dx, dy) in [(invl, 0), (-invl, 0), (0, invl), (0, -invl), (invl, invl), (invl, -invl), (-invl, -invl), (-invl, invl)]: # Allowed set of dx and dy
        pos = start + np.array([[dx], [dy]])
        if not obstacles.is_collision(pos):
            possArr += [pos]

    return possArr
"""
    start: nparray of (x, y, vx, vy, t)
    return: updated x, y, vx, vy
"""
def dynamicStateTransition(start, obstacles):
    #print(start)
    #print("__________________")
    succs = []
    dt = 0.1

    possArr = []
    k = 15
    delta = 6
    succs = []

    x0 = start[0][0]
    y0 = start[1][0]
    x = np.array([[x0], [y0]])

    t = start[4][0]

    vx = start[2][0]
    vy = start[3][0]
    v = np.array([[vx], [vy]])

    #goalx = goal.state[0][0]
    #goaly = goal.state[1][0]

    for i in range(k):
        randTheta = random.randint(0, 360)
        randTheta = math.radians(randTheta)

        u = np.array([
            [aMax * math.sin(randTheta)],
            [aMax * math.cos(randTheta)]
        ])

        dragCoeff = abs(aMax / vMax)

        a = u - dragCoeff * v

        vf = v + a*dt
        xf = x + v*dt + 1/2*a*dt**2
        tf = t + dt

        poss = np.array([
            [xf[0][0]],
            [xf[1][0]],
            [vf[0][0]],
            [vf[1][0]],
            [tf]
        ])

        if not obstacles.is_collision(poss):
            succs += [poss]

    return succs
"""
params:
    start: startNode
    end: endNode
    sparseNodes: list of sparseNodes
    path: complete path from start to end
"""
def plotPath(start, end, sparseNodes, path, cl, obstacles):
    xValsPath = [x.state[0][0] for x in path] # x/y of path
    yValsPath = [y.state[1][0] for y in path]
    print(len(xValsPath))

    xValsSparse = [x.state[0][0] for x in sparseNodes] # x/y of path
    yValsSparse = [y.state[1][0] for y in sparseNodes]

    xValsCL = [x.state[0][0] for x in cl] # x/y of path
    yValsCL = [y.state[1][0] for y in cl]

    xObsVals = [x[0] for x in obstacles] # x/y of obstacle`s
    yObsVals = [y[1] for y in obstacles]

    plt.plot(xValsPath, yValsPath) #xy of path
    plt.plot(xValsPath, yValsPath,'or') # connect the path
    plt.plot(xValsCL, yValsCL, 'o', color = 'purple') 
    plt.plot(xValsSparse, yValsSparse) #xy of sparseNodes

    plt.plot(xObsVals, yObsVals, 'o', color = "black") # X marker for obstacles
    plt.plot(start[0][0], start[1][0], 'o', color = "blue") # start marker
    plt.plot(end[0][0], end[1][0], 's', color = "green") # end marker

    dx = abs(start[0][0] - end[0][0])
    dy = abs(start[1][0] - end[1][0])
    #print("dx: %d  |  dy: %d" %(dx, dy))
    dim = dx if (dx >= dy) else dy

    plt.axis([-1, 1200, -1, 900])
    # 1200/18 x 900/18
    plt.show()
    pass

def main():
    # random.seed(30)
    obstacles = []
    for obstacle in range (50):
        y = random.randint(0, 900)
        x = random.randint(0, 1200)
        vx = 0
        vy = 0

        obstacles.append(MovingObstacle(x, y, vx, vy, 20))

    print(len(obstacles))

    # for x in range(7):
    #     for y in range(300):
    #         obstacles += [(100 + x, 100 + y)]

    # for x in range(7):
    #     for y in range(150):
    #         obstacles += [(150 + x, 70 + y)]

    start = np.array([
        [1],
        [1],
        [0],
        [0],
        [0]
    ])
    end = np.array([
        [1000],
        [800],
        [0],
        [0]
    ])

    weight = 2.0
    planner = RStarPlanner(start, end, gammaTransFunc, dynamicStateTransition, heuristic, weight, obstacles, cost)
    start = time.time()
    graphingData = planner.plan()
    elapsed = time.time() - start
    print("R* Planning took %.2f seconds" %(elapsed))
    #plotPath(*graphingData, obstacles)
    animate(graphingData[3], obstacles)

def testing():
    # start = Node(np.array([[1], [1]]), np.array([[1], [1]]), 1, 1)
    start = np.array([
        [1],
        [1],
        [3],
        [4],
        [0]
    ])


    # astar(node.parent.state, node.state, self.heuristic, self.sTransition, self.obstacleGrid)

    obstacles = Node(np.array([[1], [1]]), np.array([[1], [1]]), 1, 1)

    print(dynamicStateTransition(start, obstacles))

    # obstacles = []
    # for obstacle in range (21):
    #     y = random.randint(0, 300)
    #     x = random.randint(0, 400)

    #     for row in range(x, x+10):
    #         obstacles += [(row, y)]
    
    # obstacleGrid = stateGrid(obstacles)
    # astar(np.array([[0, 0]]).T, np.array([[40, 40]]).T, heuristic, stateTransition, obstacleGrid)
    # pass

#testing()
main()