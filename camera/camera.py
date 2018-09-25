from ball.camera_ball import CameraBall
from ball.kalman_ball import KalmanBall
from robot.camera_robot import CameraRobot
from robot.kalman_robot import KalmanRobot

from numpy import linalg as LA
import numpy as np

import util.config

class Camera:
    def __init__(self, camera_id):
        self.camera_id = camera_id
        # List of all the kalman balls in this camera
        self.kalman_balls = []
        # List of all the robot ids and all the kalman robots corresponding to that robot id in this camera
        self.kalman_robots_blue = [[] for x in range(0, util.config.max_num_robots_per_team)]
        self.kalman_robots_yellow = [[] for x in range(0, util.config.max_num_robots_per_team)]

        # TODO: Add simple transformation matrices
        #       to do scaling, skewing, rotation, translation, and shear

    # Matches the list of camera balls for this specific camera to the kalman balls
    def update_camera_balls(self, camera_balls_list, previous_world_ball):
        for kalman_ball in self.kalman_balls:
            if kalman_ball.is_unhealthy():
                self.kalman_balls.remove(kalman_ball)

        # If we don't have any ball measurements, just predict everything
        if (len(camera_balls_list) == 0):
            for kalman_ball in self.kalman_balls:
                kalman_ball.predict()

            return

        # Match specific camera balls to specific kalman balls
        # Combine the camera balls before treating as an observation for the kalman balls
        # Merge using a weighted average based on their covariance
        if (util.config.use_multi_hypothesis):
            self.update_camera_balls_MHKF(camera_balls_list, previous_world_ball)

        # Just merge all the cam balls together and apply to the single kalman filter
        else:
            self.update_camera_balls_AKF(camera_balls_list, previous_world_ball)

    # Does a Multi-Hypothesis Kalman Filter Update
    def update_camera_balls_MHKF(self, camera_balls_list, previous_world_ball):
        if (len(self.kalman_balls) == 0):
            # Average all the balls in this cameras view to init stuff
            self.kalman_balls.append(
                self.avg_pos_kalman_filter(camera_balls_list, previous_world_ball))

            return

        # Try merging some kalman filters together
        # Should check states against each other
        # If all the norms almost match up
        # Merge like we do in the world ball stuff

        # Observation is all the camera balls
        # Measurement is what we feed the kalman filters as a z_k

        # Keeps track of what the average ball measurement for this filter
        pos_measurements = np.zeros((len(self.kalman_balls), 2))
        # Time of measurement
        time_measurements = np.zeros((len(self.kalman_balls), 1))
        # Keeps track of how many observations were applied to this average measurement
        num_kf_measurements = np.zeros((len(self.kalman_balls), 1))

        # List of camera_balls used
        camera_balls_used_list = []

        for camera_ball in camera_balls_list:
            # Check all previous kalman filters
            # If close enough, add to average of that filter
            # If not close enough, create new filter
            # Try both a constant dist and a 3 sigma type thing
            for idx, kalman_ball in enumerate(self.kalman_balls):
                # Had to do this instead of np.subtract for some weird reason
                # Some matrix to array subtraction bug
                dx = kalman_ball.pos()[0] - camera_ball.pos[0]
                dy = kalman_ball.pos()[1] - camera_ball.pos[1]
                dist = np.linalg.norm([dx, dy])

                # Valid measurement
                # Add to our running average and observation count
                if (dist < util.config.multi_hypothesis_radius_cutoff):
                    x = pos_measurements[idx][0]
                    y = camera_ball.pos[0]
                    pos_measurements[idx][0] += camera_ball.pos[0]
                    pos_measurements[idx][1] += camera_ball.pos[1]
                    time_measurements[idx][0] += camera_ball.time_captured
                    num_kf_measurements[idx][0] += 1

                    camera_balls_used_list.append(camera_ball)

        # Update all filters if they have ball measurement
        # If they don't, just predict
        for idx, kalman_ball in enumerate(self.kalman_balls):

            # Get average pos
            if (num_kf_measurements[idx] > 0):
                x = pos_measurements[idx][0]
                y = num_kf_measurements[idx]
                x_avg = pos_measurements[idx][0] / num_kf_measurements[idx][0]
                y_avg = pos_measurements[idx][1] / num_kf_measurements[idx][0]
                time_avg = time_measurements[idx][0] / num_kf_measurements[idx][0]

                pos_avg = [x_avg, y_avg]

                kalman_ball.predict_and_update(time_avg, pos_avg)
            else:
                kalman_ball.predict()

        # Any balls that didn't have a update, build a new kalman ball for it
        # Get difference between lists
        unused_balls_list = set(camera_balls_list).symmetric_difference(set(camera_balls_used_list))
        unused_balls_list = list(unused_balls_list)

        for unused_ball in unused_balls_list:
            pos = unused_ball.pos
            vel = [0, 0]

            # If we have a world ball, use that vel
            if (previous_world_ball is not None):
                vel = previous_world_ball.vel

            # Add kalman ball at that pos using world ball vel if possible
            self.kalman_balls.append(
                KalmanBall(unused_ball.time_captured, pos, vel))

    # Does a Average Kalman Filter Update
    def update_camera_balls_AKF(self, camera_balls_list, previous_world_ball):
        # If we don't have any kalman balls
        # average all the camera balls
        if (len(self.kalman_balls) == 0):
            self.kalman_balls.append(
                self.avg_pos_kalman_filter(camera_balls_list, previous_world_ball))

            return

        # There is already a kalman_ball
        # Average the camera balls and use it as the observation
        pos_avg = self.avg_ball_pos(camera_balls_list)

        self.kalman_balls[0].predict_and_update(camera_balls_list[0].time_captured, pos_avg)

    def update_camera_robots(self, camera_robots_list_blue, camera_robots_list_yellow,
                                   previous_world_robot_blue, previous_world_robot_yellow):
        # Same thing as the ball, but has the added complications of bot id's
        if (util.config.use_multi_hypothesis):
            pass
        else:
            pass

        # Match specific robots to specific kalman balls with that same robot id
        # Merge anything nearby before treating it as an observation
        # OR
        # Just merge all the cam robots together and apply to the single kalman filter for that robot

    def update_camera_without_data(self):
        # Update all the kalman filters since we have no data this frame
        for kalman_ball in self.kalman_balls:
            kalman_ball.predict()

        for kalman_robot_list in self.kalman_robots_blue:
            for kalman_robot in kalman_robot_list:
                kalman_robot.predict()

        for kalman_robot_list in self.kalman_robots_blue:
            for kalman_robot in kalman_robot_list:
                kalman_robot.predict()


    # Averages all the ball positions and returns a kalman filter for that object
    # Used when there is no kalman filter to update
    def avg_pos_kalman_filter(self, camera_balls_list, previous_world_ball):
        pos_avg = self.avg_ball_pos(camera_balls_list)

        vel = [0, 0]

        # If we already have a world ball last frame, use that vel
        # for initialization
        if (previous_world_ball is not None):
            vel = previous_world_ball.vel

        return KalmanBall(camera_balls_list[0].time_captured, pos_avg, vel)

    # Average position of the ball list
    def avg_ball_pos(self, balls_list):
        pos_avg = [0, 0]

        # Do average
        for ball in balls_list:
            pos_avg = np.add(pos_avg, ball.pos)

        pos_avg = np.multiply(pos_avg, 1/len(balls_list))

        return pos_avg