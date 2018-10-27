from ball.camera_ball import CameraBall
from ball.kalman_ball import KalmanBall
from robot.camera_robot import CameraRobot
from robot.kalman_robot import KalmanRobot
import ball.process_ball_interactions

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

    # Processes bounces on the kalman ball given a list of robot positions
    def process_ball_bounce(self, world_robot_list):
        for kalman_ball in self.kalman_balls:
            new_vel = ball.process_ball_interactions.calculate_bounce(kalman_ball, world_robot_list)

            # TODO: Should explore whether we should reset the convariance
            if new_vel is not None and np.linalg.norm(new_vel, 2) > 0.1:
                kalman_ball.set_vel(new_vel)


    # Matches the list of camera balls for this specific camera to the kalman balls
    def update_camera_balls(self, calc_time, camera_balls_list, previous_world_ball):
        for kalman_ball in self.kalman_balls:
            if kalman_ball.is_unhealthy():
                self.kalman_balls.remove(kalman_ball)

        # If we don't have any ball measurements, just predict everything
        if (len(camera_balls_list) == 0):
            for kalman_ball in self.kalman_balls:
                kalman_ball.predict(calc_time)

            return

        # Match specific camera balls to specific kalman balls
        # Combine the camera balls before treating as an observation for the kalman balls
        # Merge using a weighted average based on their covariance
        if (util.config.use_multi_hypothesis):
            self.update_camera_balls_MHKF(calc_time, camera_balls_list, previous_world_ball)

        # Just merge all the cam balls together and apply to the single kalman filter
        else:
            self.update_camera_balls_AKF(calc_time, camera_balls_list, previous_world_ball)

    # Does a Multi-Hypothesis Kalman Filter Update
    def update_camera_balls_MHKF(self, calc_time, camera_balls_list, previous_world_ball):
        if (len(self.kalman_balls) == 0):
            # Average all the balls in this cameras view to init stuff
            self.kalman_balls.append(
                self.avg_ball_pos_kalman_filter(camera_balls_list, previous_world_ball))

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
                x_avg = pos_measurements[idx][0] / num_kf_measurements[idx][0]
                y_avg = pos_measurements[idx][1] / num_kf_measurements[idx][0]
                time_avg = time_measurements[idx][0] / num_kf_measurements[idx][0]

                pos_avg = [x_avg, y_avg]
                avg_ball = ball.camera_ball.CameraBall(time_avg, 100, x_avg, y_avg)

                kalman_ball.predict_and_update(calc_time, avg_ball)
            else:
                kalman_ball.predict(calc_time)

        # Any balls that didn't have a update, build a new kalman ball for it
        # Get difference between lists
        unused_balls_list = set(camera_balls_list).symmetric_difference(set(camera_balls_used_list))
        unused_balls_list = list(unused_balls_list)

        for unused_ball in unused_balls_list:

            # Add kalman ball at that pos using world ball vel if possible
            self.kalman_balls.append(
                KalmanBall(unused_ball.time_captured, unused_ball, previous_world_ball))

    # Does a Average Kalman Filter Update
    def update_camera_balls_AKF(self, calc_time, camera_balls_list, previous_world_ball):
        # If we don't have any kalman balls
        # average all the camera balls
        if (len(self.kalman_balls) == 0):
            self.kalman_balls.append(
                ball.kalman_ball.KalmanBall(calc_time,
                                            self.avg_ball_pos(camera_balls_list),
                                            previous_world_ball)

            return

        # There is already a kalman_ball
        # Average the camera balls and use it as the observation
        ball_avg = self.avg_ball_pos(camera_balls_list)

        self.kalman_balls[0].predict_and_update(calc_time, ball_avg)

    def update_camera_robots(self, calc_time, camera_robots_list_blue, camera_robots_list_yellow,
                                   previous_world_robot_blue, previous_world_robot_yellow):

        for bot_id in range(util.config.max_num_robots_per_team):
            for kalman_robot in self.kalman_robots_blue[bot_id]:
                if kalman_robot.is_unhealthy():
                    self.kalman_robots_blue[bot_id].remove(kalman_robot)

            for kalman_robot in self.kalman_robots_yellow[bot_id]:
                if kalman_robot.is_unhealthy():
                    self.kalman_robots_yellow[bot_id].remove(kalman_robot)

        # camera_robots_single_id_list is the list of observations for that specific robot id
        # previous_world_robot_single is the single robot that is the assumed robot
        for bot_id in range(util.config.max_num_robots_per_team):
            camera_robots_single_id_list_blue = list(filter(lambda x: x.bot_id == bot_id, camera_robots_list_blue))
            camera_robots_single_id_list_yellow = list(filter(lambda x: x.bot_id == bot_id, camera_robots_list_yellow))
            previous_world_robot_single_blue = previous_world_robot_blue[bot_id]
            previous_world_robot_single_yellow = previous_world_robot_yellow[bot_id]

            # Make sure not to update stuff twice
            blue_already_predicted = False
            yellow_already_predicted = False

            # Update all the kalman stuff not updated
            if (len(camera_robots_single_id_list_blue) == 0):
                for kalman_robot in self.kalman_robots_blue[bot_id]:
                    kalman_robot.predict(calc_time)

                blue_already_predicted = True

            if (len(camera_robots_single_id_list_yellow) == 0):
                for kalman_robot in self.kalman_robots_yellow[bot_id]:
                    kalman_robot.predict(calc_time)

                yellow_already_predicted = True

            # Select which observation -> KF measurement we are using
            if (util.config.use_multi_hypothesis):
                # Go through each and every robot, try to apply to list

                if not blue_already_predicted:
                    self.kalman_robots_blue[bot_id] = \
                        self.update_camera_robot_MHKF(
                            calc_time,
                            bot_id,
                            camera_robots_single_id_list_blue,
                            previous_world_robot_single_blue,
                            self.kalman_robots_blue[bot_id])

                if not yellow_already_predicted:
                    self.kalman_robots_yellow[bot_id] = \
                        self.update_camera_robot_MHKF(
                            calc_time,
                            bot_id,
                            camera_robots_single_id_list_yellow,
                            previous_world_robot_single_yellow,
                            self.kalman_robots_yellow[bot_id])
            else:

                if not blue_already_predicted:
                    self.kalman_robots_blue[bot_id] = \
                        self.update_camera_robot_AKF(
                            bot_id,
                            camera_robots_single_id_list_blue,
                            previous_world_robot_single_blue,
                            self.kalman_robots_blue[bot_id])

                if not yellow_already_predicted:
                    self.kalman_robots_yellow[bot_id] = \
                        self.update_camera_robot_AKF(
                            bot_id,
                            camera_robots_single_id_list_yellow,
                            previous_world_robot_single_yellow,
                            self.kalman_robots_yellow[bot_id])

    # Updates all the kalman filters for this specific robot id using the MHKF algorithm
    def update_camera_robot_MHKF(self,
                calc_time,
                bot_id,
                camera_robots_single_id_list,
                previous_world_robot_single,
                kalman_robots_single_id_list):
        # Go through and try to apply this observation to each kalman filter
        # Average all observations on a single kalman filter
        # Use that as the update measurement
        # If there are some left over, create a new filter at that point

        if (len(kalman_robots_single_id_list) == 0):
            # Average all the robots for this specific id and use them
            kalman_robots_single_id_list.append(
                self.avg_robot_pos_kalman_filter(
                    bot_id,
                    camera_robots_single_id_list,
                    previous_world_robot_single))

            return kalman_robots_single_id_list

        # Try merging some kalman filters together
        # Should check states against each other
        # If all the norms almost match up
        # Merge like we do in the world ball stuff

        # Observation is all the camera robots
        # Measurement is what we feed the kalman filters as a z_k

        # Keeps track of what the average ball measurement for this filter
        pos_measurements = np.zeros((len(kalman_robots_single_id_list), 3))
        # Time of measurement
        time_measurements = np.zeros((len(kalman_robots_single_id_list), 1))
        # Keeps track of how many observations were applied to this average measurement
        num_kf_measurements = np.zeros((len(kalman_robots_single_id_list), 1))

        # List of camera_robots used
        camera_robots_used_list = []

        for camera_robot in camera_robots_single_id_list:
            # Check all previous kalman filters
            # If close enough, add to average of that filter
            # If not close enough, create new filter
            # Try both a constant dist and a 3 sigma type thing
            for idx, kalman_robot in enumerate(kalman_robots_single_id_list):
                # Had to do this instead of np.subtract for some weird reason
                # Some matrix to array subtraction bug
                dx     = kalman_robot.pos()[0] - camera_robot.pos[0]
                dy     = kalman_robot.pos()[1] - camera_robot.pos[1]

                dist = np.linalg.norm([dx, dy])

                # Valid measurement
                # Add to our running average and observation count
                if (dist < util.config.multi_hypothesis_radius_cutoff):
                    pos_measurements[idx][0] += camera_robot.pos[0]
                    pos_measurements[idx][1] += camera_robot.pos[1]
                    pos_measurements[idx][2] += camera_robot.pos[2]
                    time_measurements[idx][0] += camera_robot.time_captured
                    num_kf_measurements[idx][0] += 1

                    camera_robots_used_list.append(camera_robot)

        # Update all filters if they have ball measurement
        # If they don't, just predict
        for idx, kalman_robot in enumerate(kalman_robots_single_id_list):

            # Get average pos
            if (num_kf_measurements[idx] > 0):
                x_avg     = pos_measurements[idx][0] / num_kf_measurements[idx][0]
                y_avg     = pos_measurements[idx][1] / num_kf_measurements[idx][0]
                theta_avg = pos_measurements[idx][2] / num_kf_measurements[idx][0]
                time_avg = time_measurements[idx][0] / num_kf_measurements[idx][0]

                pos_avg = [x_avg, y_avg, theta_avg]

                kalman_robot.predict_and_update(time_avg, pos_avg)
            else:
                kalman_robot.predict(calc_time)

        # Any balls that didn't have a update, build a new kalman ball for it
        # Get difference between lists
        unused_robots_list = set(camera_robots_single_id_list).symmetric_difference(set(camera_robots_used_list))
        unused_robots_list = list(unused_robots_list)

        for unused_robot in unused_robots_list:
            pos = unused_robot.pos
            vel = [0, 0, 0]

            # If we have a world robot, use that vel
            if (previous_world_robot_single is not None):
                vel = previous_world_robot_single.vel

            # Add kalman robot at that pos using world robot vel for this specific id
            kalman_robots_single_id_list.append(
                KalmanRobot(unused_robot.time_captured, pos, vel, bot_id))

        return kalman_robots_single_id_list

    def update_camera_robot_AKF(self,
                bot_id,
                camera_robots_single_id_list,
                previous_world_robot_single,
                kalman_robots_single_id_list):
        # Go through all the observations and average them all together
        # Use that as the update measurement
        # Check for no kalman filters existing
        if (len(kalman_robots_single_id_list) == 0):
            kalman_robots_single_id_list.append(
                self.avg_robot_pos_kalman_filter(
                    bot_id,
                    camera_robots_single_id_list,
                    previous_world_robot_single)
            )

            return kalman_robots_single_id_list

        pos_avg = self.avg_robot_pos(camera_robots_single_id_list)

        kalman_robots_single_id_list[0].predict_and_update(
            camera_robots_single_id_list[0].time_captured, pos_avg)

        return kalman_robots_single_id_list

    def update_camera_without_data(self, calc_time):
        # Update all the kalman filters since we have no data this frame
        for kalman_ball in self.kalman_balls:
            kalman_ball.predict(calc_time)

        for kalman_robot_list in self.kalman_robots_blue:
            for kalman_robot in kalman_robot_list:
                kalman_robot.predict(calc_time)

        for kalman_robot_list in self.kalman_robots_yellow:
            for kalman_robot in kalman_robot_list:
                kalman_robot.predict(calc_time)

    # Returns average ball of the list
    def avg_ball_pos(self, balls_list):
        pos_avg = [0, 0]
        time_avg = 0

        # Do average
        for ball in balls_list:
            pos_avg = np.add(pos_avg, ball.pos)
            time_avg = time_avg + ball.time_captured

        pos_avg = np.multiply(pos_avg, 1/len(balls_list))
        time_avg /= len(balls_list)

        ball_avg = ball.camera_ball.CameraBall(time_avg, pos_avg[0], pos_avg[1])

        return ball_avg

    # Averages all the robot positions and returns a kalman filter for that object
    # Used when there is no kalman filter to update
    def avg_robot_pos_kalman_filter(self, bot_id, camera_robots_single_id_list, previous_world_robot_single):
        pos_avg = self.avg_robot_pos(camera_robots_single_id_list)

        vel = [0, 0, 0]

        # If we already have a robot initialized for this speicifc id last frame,
        # use that vel for initialization
        if (previous_world_robot_single is not None):
            vel = previous_world_robot_single.vel

        return KalmanRobot(camera_robots_single_id_list[0].time_captured, pos_avg, vel, bot_id)

    # Average position of the robot list
    def avg_robot_pos(self, robots_list):
        pos_avg = [0, 0, 0]

        # Do average
        for robot in robots_list:
            pos_avg = np.add(pos_avg, robot.pos)

        pos_avg = np.multiply(pos_avg, 1/len(robots_list))

        return pos_avg