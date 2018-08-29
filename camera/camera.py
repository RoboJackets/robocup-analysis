from ball.camera_ball import CameraBall
from ball.kalman_ball import KalmanBall
from robot.camera_robot import CameraRobot
from robot.kalman_robot import KalmanRobot

from numpy import linalg as LA

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
    def update_camera_balls(self, camera_balls_list):

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
                self.kalman_balls.append(KalmanBall(camera_ball_pos))
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
            if (len(self.kalman_balls) == 0):
                # Use average of all the balls in the list to initialize
                return
            
            # There is already a kalman_ball
            average_pos = [] # TODO: Make 2d matrix initialized to 0
            num_balls_valid = 0

            for camera_ball in camera_balls_list:
                camera_ball_pos = [] # TODO: Make 2d matrix of x, y
                dist = LA.norm(self.kalman_balls[0].state - camera_ball_pos)

                if (dist > util.config.single_kalman_radius_cutoff):
                    camera_balls_list.remove(camera_ball)
                    continue

                average_pos += camera_ball_pos
                num_balls_valid += 1

            self.kalman_balls[0].predict_and_update(average_pos)
        

    def update_camera_robots(self, camera_robots_list_blue, camera_robots_list_yellow):
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