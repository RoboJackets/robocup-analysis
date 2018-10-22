import numpy as np
import matplotlib.pylab as plt
from matplotlib.animation import FuncAnimation

dt = 0.1

def animate(states, obstacles):
    fig, ax = plt.subplots()

    line, = plt.plot([], [], 'ro', animated = True)
    obstacles_line, = plt.plot([], [], 'bo', animated = True, markersize = 5)

    def init():
        ax.set_xlim(0, 1200)
        ax.set_ylim(0, 900)
        return line, obstacles_line,

    def update(t):
        i = int(np.round(t / dt))
        line.set_data([node.state[0][0] for node in states[max(0, i - 40):i + 1]], [node.state[1][0] for node in states[max(0, i - 40):i + 1]])
        obstacles_line.set_data([obs.x0 + t * obs.vx0 for obs in obstacles], [obs.y0 + t * obs.vy0 for obs in obstacles])
        return line, obstacles_line,


    #print("STATES: %s", states)
    frames = [node.state[4][0] for node in states]
    #print(frames)
    anim = FuncAnimation(fig, update, frames, init_func = init, blit = True, interval = 10)
    plt.show()

def main():
    states = [np.asmatrix([0, 0, 0, 0, 0]).T]
    obstacles = [[np.asmatrix([400, 400, -15, -35, 0]).T, np.asmatrix([10, 10, 5, 20, 0]).T]]

    for _ in range(400):
        states.append(states[-1] + np.asmatrix([.5, .5, 0.0, 0.0, .01]).T)
        prev_obstacles = obstacles[-1]
        new_obstacles = [x + np.asmatrix([x[2, 0], x[3, 0], 0, 0, 1]).T * dt for x in prev_obstacles]
        obstacles.append(new_obstacles)

    animate(states, obstacles)

if __name__ == '__main__':
    main()
