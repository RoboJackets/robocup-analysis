from ball.camera_ball import CameraBall
from ball.kalman_ball import KalmanBall
from robot.camera_robot import CameraRobot
from robot.kalman_robot import KalmanRobot

class Camera:
    def __init__(self, camera_id):
        self.camera_id = camera_id
        # List of all the kalman balls in this camera
        self.kalman_balls = []
        # List of all the robot ids and all the kalman robots corresponding to that robot id in this camera
        self.kalman_robots = [[]] # TODO: Initialize this correctly

    # Matches the list of camera balls for this specific camera to the kalman balls
    def match_camera_balls(self, camera_balls_list, camera_id):
        if (self.camera_id != camera_id):
            return
        
        # Match specific camera balls to specific kalman balls
        # Combine the camera balls before treating as an observation for the kalman balls
        # Merge using a weighted average based on their covariance

    def match_camera_robots(self, camera_robots_list, camera_id):
        if (self.camera_id != camera_id):
            return

        # Match specific robots to specific kalman balls with that same robot id
        # Merge anything nearby before treating it as an observation
