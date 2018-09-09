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
            if (len(self.kalman_balls) == 0):
                camera_ball_pos = [] # TODO: Make 2d matrix of x,y
                self.kalman_balls.append(KalmanBall(0, 0, 0, 0, 0))
                camera_balls_list.pop(0)

            for camera_ball in camera_balls_list:
                # Check all previous kalman filters
                # If close enough, add to average of that filter
                # If not close enough, create new filter
                pass

            for kalman_ball in self.kalman_balls:
                # Update all filters if they have ball measurement
                # If they don't, just predict
                pass

        # Just merge all the cam balls together and apply to the single kalman filter
        else:
            # If we don't have any kalman balls
            # average all the camera balls
            # TODO: Get last frames velocity as initialization
            if (len(self.kalman_balls) == 0):
                x_avg = 0
                y_avg = 0

                for camera_ball in camera_balls_list:
                    x_avg += camera_ball.x
                    y_avg += camera_ball.y
                
                x_avg /= len(camera_balls_list)
                y_avg /= len(camera_balls_list)

                x_vel = 0
                y_vel = 0

                if (previous_world_ball is not None):
                    x_vel = previous_world_ball.x_vel
                    y_vel = previous_world_ball.y_vel
                
                self.kalman_balls.append(KalmanBall(camera_balls_list[0].time, x_avg, y_avg, x_vel, y_vel))

                return
            
            # There is already a kalman_ball
            # Average the camera balls and use it as the observation
            average_pos_x = 0
            average_pos_y = 0

            for camera_ball in camera_balls_list:
                average_pos_x += camera_ball.x
                average_pos_y += camera_ball.y

            average_pos_x /= len(camera_balls_list)
            average_pos_y /= len(camera_balls_list)

            if len(camera_balls_list) > 0:
                self.kalman_balls[0].predict_and_update(camera_balls_list[0].time, average_pos_x, average_pos_y)
            else:
                self.kalman_balls[0].predict()
        

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