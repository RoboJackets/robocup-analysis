# Fakes output for a set of cameras each frame
from camera.fake_single_camera_output import FakeSingleCameraOutput
import util.config

import numpy as np

import math

class FakeSSLVisionOutput:
    def __init__(self):
        self.field_width = util.config.field_width
        self.field_length = util.config.field_length
        self.num_cameras_width = util.config.num_cameras_width
        self.num_cameras_length = util.config.num_cameras_length
        self.camera_percent_overlap = util.config.camera_percent_overlap

        self.cameras = []

        camera_width = self.field_width / self.num_cameras_width
        camera_length = self.field_length / self.num_cameras_length

        for i in range(self.num_cameras_width):
            for j in range(self.num_cameras_length):
                camera_id = i*self.num_cameras_length + j

                camera_lower_left_corner = (
                    i*camera_width - self.field_width / 2,
                    j*camera_length
                )

                camera_upper_right_corner = (
                    (i+1)*camera_width - self.field_width / 2,
                    (j+1)*camera_length
                )

                self.cameras.append(
                    FakeSingleCameraOutput(camera_id, camera_lower_left_corner, camera_upper_right_corner)
                )

    # Returns list of frames given the current time for all cameras
    def get_frames(self, time):
        pt1 = np.array([-2.5, 2.5])
        pt2 = np.array([2.5, 5.5])
        line = pt2 - pt1
        line_len = np.linalg.norm(line)
        ball_pos = ((util.config.sim_ball_speed * time) % line_len) * line / line_len + pt1

        pt1 = np.array([-3, 3])
        pt2 = np.array([3, 3])
        line = pt2 - pt1
        line_len = np.linalg.norm(line)
        bot_pos = ((util.config.sim_ball_speed * time) % line_len) * line / line_len + pt1

        blue_robots = [[bot_pos[0], bot_pos[1], 0, 0]]

        yellow_robots = []

        frames = []
        for camera in self.cameras:
            frames.append(camera.get_frame(time, ball_pos, yellow_robots, blue_robots))

        return frames
