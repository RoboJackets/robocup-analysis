import pygame
import numpy as np

class Visualizer:
    def __init__(self):
        self.resolution = (800, 600)
        self.display = pygame.display.set_mode(self.resolution)
        pygame.display.set_caption("RoboCup Model Simulator")
        self.close = False
        self.keys = set([])
        self.ppm = 200
        self.radius = 0.09

    def update_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.close = True
            elif event.type == pygame.KEYDOWN:
                self.keys.add(event.key)
            elif event.type == pygame.KEYUP:
                self.keys.remove(event.key)

    def extract_wheel_inputs(self):
        u = np.asmatrix(np.zeros((4, 1)))
        max_torque = 0.780
        if ord('1') in self.keys:
            u[0, 0] = max_torque
        if ord('2') in self.keys:
            u[0, 0] = -max_torque
        if ord('3') in self.keys:
            u[1, 0] = max_torque
        if ord('4') in self.keys:
            u[1, 0] = -max_torque
        if ord('q') in self.keys:
            u[2, 0] = max_torque
        if ord('w') in self.keys:
            u[2, 0] = -max_torque
        if ord('e') in self.keys:
            u[3, 0] = max_torque
        if ord('r') in self.keys:
            u[3, 0] = -max_torque
        return u

    def extract_goals(self):
        r = np.asmatrix(np.zeros((3, 1)))
        speed_max = 5.0
        arate_max = 6.0
        if ord('w') in self.keys:
            r[1, 0] = speed_max
        if ord('s') in self.keys:
            r[1, 0] = -speed_max
        if ord('a') in self.keys:
            r[0, 0] = -speed_max
        if ord('d') in self.keys:
            r[0, 0] = speed_max
        if ord('q') in self.keys:
            r[2, 0] = arate_max
        if ord('e') in self.keys:
            r[2, 0] = -arate_max
        return r

    def draw(self, robot_coords, goal_point):
        screen_coords = self.to_screen_coords(robot_coords)
        heading = robot_coords[2]
        line_end_coords = (
            screen_coords[0] + int(self.ppm * self.radius * 4 * np.cos(heading)),
            screen_coords[1] + int(self.ppm * self.radius * 4 * np.sin(heading))
        )
        radius = int(self.ppm * self.radius)

        self.display.fill((0, 0, 0))
        pygame.draw.circle(self.display, (255, 0, 0), screen_coords, radius)
        pygame.draw.aaline(self.display, (255, 255, 255),
                           screen_coords, line_end_coords, 10)

        rscreen_coords = self.to_screen_coords(goal_point)
        rline_end_coords = (
            rscreen_coords[0] + int(self.ppm * self.radius * 4 * np.cos(goal_point[2])),
            rscreen_coords[1] + int(self.ppm * self.radius * 4 * np.sin(goal_point[2]))
        )
        pygame.draw.aaline(self.display, (0, 255, 0),
                           rscreen_coords, rline_end_coords, 10)
        pygame.draw.circle(self.display, (0, 255, 0),
                           rscreen_coords, 5)
        pygame.display.update()

    def to_screen_coords(self, pos):
        return (int(pos[0] * self.ppm + self.resolution[0] / 2),
                int(self.resolution[1] / 2 - pos[1] * self.ppm))
