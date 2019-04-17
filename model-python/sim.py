import pygame
import model
import numpy as np
from controller import Controller
import vis


def main():
    clock = pygame.time.Clock()

    pos = np.asmatrix([0, 0, 0.]).T
    vel = np.asmatrix([0, 0, 0.]).T
    visualizer = vis.Visualizer()

    fps = 60
    dt = 1.0 / fps
    t = 0.0
    i = 0

    controller = Controller(dt)

    while not visualizer.close:
        visualizer.update_events()
        v = 1.5
        rx = np.asmatrix([np.sin(v * t), np.cos(v * t), v * t]).T
        rv = v * np.asmatrix([np.cos(v * t), -np.sin(v * t), 1]).T
        ra = v ** 2 * np.asmatrix([-np.sin(v * t), -np.cos(v * t), 0]).T

        u = controller.control(pos, vel, rx, rv, ra)

        vdot = model.forward_dynamics(vel, u, pos[2, 0])
        visualizer.draw(pos, rx)

        pos += dt * vel + 0.5 * vdot * dt ** 2
        vel += dt * vdot

        clock.tick(60)
        t += dt
        i += 1
        if t > 20:
            t = 0.0
            i = 0
            pos = np.asmatrix([0, 1, 3.]).T
            vel = np.asmatrix([1, 0, 1.]).T
            controller.reset()


if __name__ == '__main__':
    main()
