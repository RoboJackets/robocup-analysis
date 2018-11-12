import numpy as np
import matplotlib.pylab as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import CirclePolygon
import math
import csv

dt = 0.1

def get_spline_function(filename):
    with open(filename, "r") as coeffs_file:
        tf = float(coeffs_file.readline())
        coefficients = np.reshape(np.array([float(line) for line in coeffs_file]), (3, 4))
        def spline(t):
            bases = np.array([[1.0, t, t ** 2.0, t ** 3.0]]).T
            result = np.dot(coefficients, bases)
            return (result[0, 0], result[1, 0], result[2, 0])
        return tf, spline

def animate(states):
    fig, ax = plt.subplots()

    tf, spline = get_spline_function('coefficients.txt')
    ts = np.linspace(0, tf, int(tf * 100) + 1)

    line, = plt.plot([], [], 'r', animated = True)
    dot, = plt.plot([], [], 'bo', animated = True)
    plt.plot([spline(t)[0] for t in ts], [spline(t)[1] for t in ts])

    def init():
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        return line, dot,

    def update(t):
        x, y, th = spline(t)
        line.set_data([x, x + 0.5 * math.cos(th)], [y, y + 0.5 * math.sin(th)])
        dot.set_data([x], [y])
        return line, dot,

    anim = FuncAnimation(fig, update, frames=np.linspace(0, tf, int(tf * 100) + 1), init_func = init, blit = True, interval = 10)
    plt.show()

def main():
    states = []

    animate(states)

if __name__ == '__main__':
    main()
