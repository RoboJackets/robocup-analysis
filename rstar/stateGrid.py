import numpy as np
import math

class MovingObstacle():
    def __init__(self, x, y, vx, vy, radius):
        self.x0 = x
        self.y0 = y
        self.vx0 = vx
        self.vy0 = vy
        self.radius = radius

    def is_collision(self, x, y, t):
        dx = self.x0 + self.vx0 * t - x
        dy = self.y0 + self.vy0 * t - y
        return math.sqrt(dx ** 2 + dy ** 2) < self.radius

class Obstacles():
    def __init__(self, obstacles):
        self.obstacles = obstacles
    
    def is_collision(self, state):
        return False
        # for obstacle in self.obstacles:
        #     if obstacle.is_collision(state[0], state[1], state[4]):
        #         return True
        # return False

# class stateGrid:
#     """
#     items: list of States (x, y, vx, vy, t)
#     """
#     def __init__(self, items):
#         self.grid = []
#         self.addToGrid(items)

#     # 67 x 50 grid
#     def discretize(self, state):
#         return np.array([
#             [int(state[0][0]/18)],
#             [int(state[1][0]/18)],
#             [state[2][0]],
#             [state[3][0]],
#             [round(state[4][0] / 0.1)]
#         ])

#     def updateObstacles(self, time):
#         for obstacle in self.grid:
#             dt = time - obstacle[4][0]
#             dx = (dt * obstacle[2][0]) / 18
#             dy = (dt * obstacle[3][0]) / 18 

#             obstacle = self.discretize(np.array([
#                 [obstacle[0][0] + dx],
#                 [obstacle[1][0] + dy],
#                 [obstacle[2][0]],
#                 [obstacle[3][0]],
#                 [obstacle[4][0]]
#             ]))

#     def addToGrid(self, items):
        
#         for obstacle in items:
#             self.grid += [self.discretize(obstacle)]
    
#     def isCollision(self, state):
#         # given a state of the robot at some time
#         # we have set of obstacles
#         # check if
#         #print("Called is collision with: %s" %(state))
#         self.updateObstacles(state[4][0])

#         disc = self.discretize(state)
#         #print("Disc is: %s" %(disc)) 

#         t = int(disc[4][0])
#         x = int(disc[0][0])
#         y = int(disc[1][0])

#         #return False
#         for obstacle in self.grid:
#             collision = obstacle[0][0] == x and obstacle[1][0] == y
#             if collision:
#                 print("OOF")
#                 return True

#         return False