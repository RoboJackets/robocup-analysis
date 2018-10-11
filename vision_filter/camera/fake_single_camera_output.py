import numpy as np

# Fakes the output for a single camera given the true ball position
from camera.frame import Frame
from ball.camera_ball import CameraBall
from robot.camera_robot import CameraRobot

import util.config

class FakeSingleCameraOutput:
    def __init__(self, camera_id, lower_left_corner, upper_right_corner):
        self.camera_id = camera_id
        self.lower_left_corner = lower_left_corner
        self.upper_right_corner = upper_right_corner
        self.frame_number = 0

    # Ball (x, y) tuple
    # Robot (x, y, theta, bot_id) tuple
    def get_frame(self, time, ball_position, robots_yellow_position, robots_blue_position):
        viewed_balls = []
        viewed_robots_yellow = []
        viewed_robots_blue = []

        ball_confidence = 100
        robot_confidence = 100

        # In camera view?
        if (self.item_in_view(ball_position)):
            # TODO: Add in the multiple ball detections
            viewed_balls.append(
                CameraBall(time, ball_confidence, ball_position[0], ball_position[1])#*np.random.normal(ball_position, util.config.camera_noise))
            )

        for robot_yellow in robots_yellow_position:
            if (self.item_in_view(robot_yellow)):
                rand_pos = robot_yellow[0:3] #np.random.normal(robot_yellow[0:3], util.config.camera_noise)
                viewed_robots_yellow.append(
                    CameraRobot(time, robot_confidence, rand_pos[0], rand_pos[1], rand_pos[2], robot_yellow[3])
                )

        for robot_blue in robots_blue_position:
            if (self.item_in_view(robot_blue)):
                rand_pos = robot_blue[0:3] #np.random.normal(robot_blue[0:3], util.config.camera_noise)
                viewed_robots_blue.append(
                    CameraRobot(time, robot_confidence, rand_pos[0], rand_pos[1], rand_pos[2], robot_blue[3])
                )

        self.frame_number += 1

        return Frame(self.frame_number, time, time, self.camera_id, viewed_balls, viewed_robots_yellow, viewed_robots_blue)

    def item_in_view(self, pos):
        return (pos[0] > self.lower_left_corner[0] and
                pos[0] <= self.upper_right_corner[0] and
                pos[1] > self.lower_left_corner[1] and
                pos[1] <= self.upper_right_corner[1])
