# Ball in the world space (not single camera space) which consists of one or more merged kalman balls
from ball.kalman_ball import KalmanBall
import util.config
import numpy as np
import math

class WorldBall:
    def __init__(self, kalman_balls):
        pos_avg              = [0, 0]
        vel_avg              = [0, 0]
        pos_measurement_avg  = [0, 0]
        time_measurement_avg = 0
        total_filter_pos_weight = 0
        total_filter_vel_weight = 0

        # This should never happen, but it's buggy right now
        if len(kalman_balls) == 0:
            print('WorldBall::ERROR::NoKalmanBalls?')
            self.pos = pos_avg
            self.vel = vel_avg

            return

        # Get x, y, x_vel, y_vel uncertantity, sqrt(covariance along diagonal)
        # get 1 / health of filter
        # Multiple l2 norm of pos (xy) / vel (xy) uncertantity by health
        # Bring to the -1.5ish power
        # This add these as the position and velocity uncertanty

        # Add all of the states from each camera multiplied by that l2 norm to the -1.5ish power
        # Divide by the total found in the first step

        for ball in kalman_balls:
            # Get the covariance of everything
            #   How well we can predict the next measurement
            cov = ball.filter.P_k_k
            x_cov     = cov.item((0,0))
            x_vel_cov = cov.item((1,1))
            y_cov     = cov.item((2,2))
            y_vel_cov = cov.item((3,3))

            # Std dev of each state value
            #   Lower std dev means we have a better idea of the true value
            x_uncer     = math.sqrt(x_cov)
            x_vel_uncer = math.sqrt(x_vel_cov)
            y_uncer     = math.sqrt(y_cov)
            y_vel_uncer = math.sqrt(y_vel_cov)

            # Inversly proportional to how many times this filter has had camera updates
            filter_uncertantity = 1 / ball.health

            # How good of position or velocity measurement in total
            pos_uncer = math.sqrt(x_uncer*x_uncer + y_uncer*y_uncer)
            vel_uncer = math.sqrt(x_vel_uncer*x_vel_uncer + y_vel_uncer*y_vel_uncer)

            # Combines the filters trust in the data with our trust in the filter
            # Inverts the results because we want a more certain filter to have a higher weight
            # Applies a slight non-linearity with the power operation so the difference between numbers is more pronounced
            filter_pos_weight = math.pow(pos_uncer * filter_uncertantity, -util.config.ball_merger_power)
            filter_vel_weight = math.pow(vel_uncer * filter_uncertantity, -util.config.ball_merger_power)

            # Apply the weighting to all the estimates
            state = ball.filter.x_k_k
            pos_avg             = np.add(pos_avg,             np.multiply(filter_pos_weight, [state.item(0), state.item(2)]))
            vel_avg             = np.add(vel_avg,             np.multiply(filter_vel_weight, [state.item(1), state.item(3)]))
            pos_measurement_avg = np.add(pos_measurement_avg, np.multiply(filter_pos_weight, ball.past_measurement[1]))
            time_measurement_avg = time_measurement_avg + ball.past_measurement[0]

            total_filter_pos_weight += filter_pos_weight
            total_filter_vel_weight += filter_vel_weight

        # These should be greater than zero always since it's basically a 1 over a vector magnitude
        if (total_filter_pos_weight <= 0 or total_filter_vel_weight <= 0):
            print('WorldBall::ERROR::WeightsAreLTZero')

        # Scale back to the normal values
        pos_avg              = np.multiply(pos_avg,             1/total_filter_pos_weight)
        vel_avg              = np.multiply(vel_avg,             1/total_filter_vel_weight)
        pos_measurement_avg  = np.multiply(pos_measurement_avg, 1/total_filter_pos_weight)
        time_measurement_avg = time_measurement_avg / len(kalman_balls)

        self.pos = pos_avg
        self.vel = vel_avg
        self.pos_cov = total_filter_pos_weight / len(kalman_balls)
        self.vel_cov = total_filter_vel_weight / len(kalman_balls)
        self.past_measurement = [time_measurement_avg, pos_measurement_avg]