from camera.camera import Camera
from ball.world_ball import WorldBall
from robot.world_robot import WorldRobot
import util.config
import matplotlib.pyplot as plt
import matplotlib

class World:
    def __init__(self):
        # Create list of cameras
        self.cameras = [None for x in range(0, util.config.max_num_cameras)]

        # Create list world robots / ball
        self.world_ball = None
        self.world_robots_blue = [None] * util.config.max_num_robots_per_team
        self.world_robots_yellow = [None] * util.config.max_num_robots_per_team

        # Plotting stuff
        self.setup_plots()

    def update_with_camera_frame(self, frame_list):
        self.plot_frames(frame_list)

        self.calculate_ball_bounce()

        # Get list of camera frames to update
        # Update balls and robots corresponding to each camera frame
        # Update other stuff when we have no frames
        updated_camera_ids = []
        for frame in frame_list:
            camera_id = frame.camera_id

            if self.cameras[camera_id] is None:
                self.cameras[camera_id] = Camera(camera_id)

            self.cameras[camera_id].update_camera_balls(frame.camera_balls, self.world_ball)
            self.cameras[camera_id].update_camera_robots(frame.camera_robots_blue,
                                                         frame.camera_robots_yellow,
                                                         self.world_robots_blue,
                                                         self.world_robots_yellow)

            updated_camera_ids.append(camera_id)

        for i in range(0, util.config.max_num_cameras):
            if (self.cameras[i] is not None and
                i not in updated_camera_ids):
                self.cameras[i].update_camera_without_data()

        self.update_world_objects()

    def update_without_camera_frame(self):
        self.calculate_ball_bounce()

        for camera in self.cameras:
            if camera is not None:
                camera.update_without_camera_frame()

        self.update_world_objects()

    def calculate_ball_bounce(self):
        pass

    def update_world_objects(self):
        # Take best kalman filter set for each camera (ball and one for each robot)
        # Average all of them together based on the covariance

        # Reset everything
        self.world_ball = None
        self.world_robots_blue = [[] for x in range(0, util.config.max_num_robots_per_team)]
        self.world_robots_yellow = [[] for x in range(0, util.config.max_num_robots_per_team)]

        kalman_ball_list = []
        robot_blue_list = [[] for x in range(0, util.config.max_num_robots_per_team)]
        robot_yellow_list = [[] for x in range(0, util.config.max_num_robots_per_team)]

        for camera in self.cameras:
            if camera is not None:
                kalman_balls  = camera.kalman_balls
                blue_robots   = camera.kalman_robots_blue
                yellow_robots = camera.kalman_robots_yellow

                if len(kalman_balls) > 0:
                    kalman_ball_list.append(kalman_balls[0])

                # for robot in robot list
                # check length and take top for each robot
                # Combine together
                for idx, blue_robots_list in enumerate(blue_robots):
                    if len(blue_robots_list) > 0:
                        robot_blue_list[idx].append(blue_robots_list[0])

                for idx, yellow_robots_list in enumerate(yellow_robots):
                    if len(yellow_robots_list) > 0:
                        robot_yellow_list[idx].append(yellow_robots_list[0])

        # Do merger

        if len(kalman_ball_list) > 0:
            self.world_ball = WorldBall(kalman_ball_list)

        self.world_robots_blue = [None] * util.config.max_num_robots_per_team
        self.world_robots_yellow = [None] * util.config.max_num_robots_per_team

        for idx, robot_list in enumerate(robot_blue_list):
            if len(robot_list) > 0:
                self.world_robots_blue[idx] = WorldRobot(idx, robot_list)

        for idx, robot_list in enumerate(robot_yellow_list):
            if len(robot_list):
                self.world_robots_yellow[idx] = WorldRobot(idx, robot_list)

    def setup_plots(self):
        self.figure, self.ax = plt.subplots()
        self.camera_ball_line, self.world_ball_line, self.robot_line = self.ax.plot([],[], 'ro', [],[], 'bo', [],[], 'go')
        self.ax.axis('scaled')
        self.ax.axis([-util.config.field_width / 2, util.config.field_width / 2,
                                                 0,     util.config.field_length])
        self.camera_ball_line.set_label('Camera ball')
        self.world_ball_line.set_label('World ball')
        self.robot_line.set_label('Robot')
        self.ax.legend()
        self.figure.show()

    def plot_frames(self, frames):
        ball_pos_x = []
        ball_pos_y = []
        bot_pos_x = []
        bot_pos_y = []

        for frame in frames:
            if len(frame.camera_balls) > 0:
                for ball in frame.camera_balls:
                    ball_pos_x.append(ball.pos[0])
                    ball_pos_y.append(ball.pos[1])

            if len(frame.camera_robots_blue) > 0:
                bot_pos_x.append(frame.camera_robots_blue[0].pos[0])
                bot_pos_y.append(frame.camera_robots_blue[0].pos[1])


        self.camera_ball_line.set_xdata(ball_pos_x)
        self.camera_ball_line.set_ydata(ball_pos_y)

        if (self.world_ball is not None):
            self.world_ball_line.set_xdata(self.world_ball.pos[0])
            self.world_ball_line.set_ydata(self.world_ball.pos[1])

        self.robot_line.set_xdata(bot_pos_x)
        self.robot_line.set_ydata(bot_pos_y)

        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
