# Ball in the world space (not single camera space) which consists of one or more merged kalman balls
from ball.kalman_ball import KalmanBall
import numpy as np

class WorldBall:
    def __init__(self, kalman_balls):
        x_avg = 0
        y_avg = 0
        x_vel_avg = 0
        y_vel_avg = 0

        if (len(kalman_balls) > 0):
            total_norm = 0
        else:
            total_norm = 1
        norm_list = []

        ball = kalman_balls[0] #for ball in kalman_balls:
        if len(kalman_balls) > 1:
            ball = kalman_balls[1]
        state = ball.filter.x_k_k
        cov = ball.filter.P_k_k
        norm = ball.health
        norm = norm*norm
        
        x_avg     += norm * state.item(0)
        x_vel_avg += norm * state.item(1)
        y_avg     += norm * state.item(2)
        y_vel_avg += norm * state.item(3)
        total_norm += norm
        norm_list.append(norm)

        x_avg     /= total_norm
        x_vel_avg /= total_norm
        y_avg     /= total_norm
        y_vel_avg /= total_norm

        if len(kalman_balls) > 1:
            print(norm_list)
            #print(cov_list)

        self.x = x_avg
        self.y = y_avg
        self.x_vel = x_vel_avg
        self.y_vel = y_vel_avg

        self.pos = (self.x, self.y)
        self.vel = (self.x_vel, self.y_vel)
