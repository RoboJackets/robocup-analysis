# Ball in the world space (not single camera space) which consists of one or more merged kalman balls
from robot.kalman_robot import KalmanRobot
import util.config
import numpy as np
import math

class WorldRobot:
    def __init__(self, bot_id, kalman_robots):
        # Filter out all robots that aren't this specific robot id
        #kalman_robots = list(filter(lambda x: x.bot_id == bot_id, kalman_robots))

        pos_avg = [0, 0]
        vel_avg = [0, 0]
        theta_avg = 0
        omega_avg = 0

        total_filter_pos_weight = 0
        total_filter_vel_weight = 0
        total_filter_theta_weight = 0
        total_filter_omega_weight = 0

        # This should never happen, but it's buggy right now
        if len(kalman_robots) == 0:
            print('WorldRobots::ERROR::NoKalmanRobots?')
            self.pos = [pos_avg[0], pos_avg[1], theta_avg]
            self.vel = [vel_avg[0], vel_avg[1], omega_avg]

            return

        # Get x, y, x_vel, y_vel, theta, omega uncertantity, sqrt(covariance along diagonal)
        # get 1 / health of filter
        # Multiple l2 norm of pos (xy) / vel (xy) / theta / omega uncertantity by health
        # Bring to the -1.5ish power
        # This add these as the position and velocity uncertanty

        # Add all of the states from each camera multiplied by that l2 norm to the -1.5ish power
        # Divide by the total found in the first step

        for robot in kalman_robots:
            # Get the covariance of everything
            #   How well we can predict the next measurement
            cov = robot.filter.P_k_k
            x_cov     = cov.item((0,0))
            x_vel_cov = cov.item((1,1))
            y_cov     = cov.item((2,2))
            y_vel_cov = cov.item((3,3))
            theta_cov = cov.item((4,4))
            omega_cov = cov.item((5,5))

            # Std dev of each state value
            #   Lower std dev means we have a better idea of the true value
            x_uncer     = math.sqrt(x_cov)
            x_vel_uncer = math.sqrt(x_vel_cov)
            y_uncer     = math.sqrt(y_cov)
            y_vel_uncer = math.sqrt(y_vel_cov)
            theta_uncer = math.sqrt(theta_cov)
            omega_uncer = math.sqrt(omega_cov)

            # Inversly proportional to how many times this filter has had camera updates
            filter_uncertantity = 1 / robot.health

            # How good of position or velocity measurement in total
            pos_uncer = math.sqrt(x_uncer*x_uncer + y_uncer*y_uncer)
            vel_uncer = math.sqrt(x_vel_uncer*x_vel_uncer + y_vel_uncer*y_vel_uncer)
            # Theta and omega are already in their correct forms

            # Combines the filters trust in the data with our trust in the filter
            # Inverts the results because we want a more certain filter to have a higher weight
            # Applies a slight non-linearity with the power operation so the difference between numbers is more pronounced
            filter_pos_weight   = math.pow(pos_uncer * filter_uncertantity,   -util.config.robot_merger_power)
            filter_vel_weight   = math.pow(vel_uncer * filter_uncertantity,   -util.config.robot_merger_power)
            filter_theta_weight = math.pow(theta_uncer * filter_uncertantity, -util.config.robot_merger_power)
            filter_omega_weight = math.pow(omega_uncer * filter_uncertantity, -util.config.robot_merger_power)

            # Apply the weighting to all the estimates
            state = robot.filter.x_k_k
            pos_avg   = np.add([pos_avg[0], pos_avg[1]],   np.multiply(filter_pos_weight,   [state.item(0), state.item(2)]))
            vel_avg   = np.add([vel_avg[0], vel_avg[1]],   np.multiply(filter_vel_weight,   [state.item(1), state.item(3)]))
            theta_avg = np.add(theta_avg, np.multiply(filter_theta_weight, state.item(4)))
            omega_avg = np.add(omega_avg, np.multiply(filter_omega_weight, state.item(5)))

            total_filter_pos_weight += filter_pos_weight
            total_filter_vel_weight += filter_vel_weight
            total_filter_theta_weight += filter_theta_weight
            total_filter_omega_weight += filter_omega_weight

        # These should be greater than zero always since it's basically a 1 over a vector magnitude
        if (total_filter_pos_weight <= 0 or
            total_filter_vel_weight <= 0 or
            total_filter_theta_weight <= 0 or
            total_filter_omega_weight <= 0):

            print('WorldRobot::ERROR::WeightsAreLTZero')

        # Scale back to the normal values
        pos_avg = np.multiply(pos_avg, 1/total_filter_pos_weight)
        vel_avg = np.multiply(vel_avg, 1/total_filter_vel_weight)
        theta_avg = np.multiply(theta_avg, 1/total_filter_theta_weight)
        omega_avg = np.multiply(omega_avg, 1/total_filter_omega_weight)

        self.pos = [pos_avg[0], pos_avg[1], theta_avg]
        self.vel = [vel_avg[0], vel_avg[1], omega_avg]