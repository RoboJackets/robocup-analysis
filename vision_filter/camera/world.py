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

    def update_with_camera_frame(self, calc_time, frame_list):
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

            self.cameras[camera_id].update_camera_balls(calc_time, frame.camera_balls, self.world_ball)
            self.cameras[camera_id].update_camera_robots(calc_time,
                                                         frame.camera_robots_blue,
                                                         frame.camera_robots_yellow,
                                                         self.world_robots_blue,
                                                         self.world_robots_yellow)

            updated_camera_ids.append(camera_id)

        for i in range(0, util.config.max_num_cameras):
            if (self.cameras[i] is not None and
                i not in updated_camera_ids):
                self.cameras[i].update_camera_without_data(calc_time)

        self.update_world_objects()

    def update_without_camera_frame(self, calc_time):
        self.calculate_ball_bounce()

        for camera in self.cameras:
            if camera is not None:
                camera.update_without_camera_frame(calc_time)

        self.update_world_objects()

    def calculate_ball_bounce(self):
        robot_list = self.world_robots_blue + self.world_robots_yellow
        for camera in self.cameras:
            if (camera is not None):
                camera.process_ball_bounce(robot_list)

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
            if len(robot_list) > 0:
                self.world_robots_yellow[idx] = WorldRobot(idx, robot_list)

    def setup_plots(self):
        self.figure, self.ax = plt.subplots()
        self.camera_ball_line, self.world_ball_line, \
            self.camera_robot_line, self.world_robot_blue_line, \
            self.world_robot_yellow_line = \
                self.ax.plot([],[], 'ro', [],[], 'go', [],[], 'co', [],[], 'bo', [],[], 'yo')
        self.ax.axis('scaled')
        self.ax.axis([ -util.config.field_width / 2,  util.config.field_width / 2,
                      -util.config.field_length / 2, util.config.field_length / 2])
        self.camera_ball_line.set_label('Camera ball')
        self.world_ball_line.set_label('World ball')
        self.camera_robot_line.set_label('Camera Robot')
        self.world_robot_blue_line.set_label('World Blue Robot')
        self.world_robot_yellow_line.set_label('World Yellow Robot')
        self.ax.legend()
        self.figure.show()

        self.figure2, self.ax2 = plt.subplots()
        self.ball_vel_x_line, self.ball_vel_y_line = self.ax2.plot([],[], 'r', [],[], 'b')
        self.ball_vel_x_line.set_label('Ball Vel X')
        self.ball_vel_y_line.set_label('Ball Vel Y')
        self.ax2.legend()
        self.figure2.show()

        self.figure3, self.ax3 = plt.subplots()
        self.ball_pos_cov_line, self.ball_vel_cov_line = self.ax3.plot([],[], 'r', [],[], 'b')
        self.ball_pos_cov_line.set_label('Ball Pos Cov')
        self.ball_vel_cov_line.set_label('Ball Vel Cov')
        self.ax3.legend()
        self.figure3.show()

        self.world_ball_vel_x = [0]
        self.world_ball_vel_y = [0]
        self.world_ball_pos_cov = [0]
        self.world_ball_vel_cov = [0]
        self.time = [0]

    def plot_frames(self, frames):
        ball_pos_x = []
        ball_pos_y = []
        camera_bot_pos_x = []
        camera_bot_pos_y = []
        world_robot_blue_pos_x = []
        world_robot_blue_pos_y = []
        world_robot_yellow_pos_x = []
        world_robot_yellow_pos_y = []

        for frame in frames:
            if len(frame.camera_balls) > 0:
                for ball in frame.camera_balls:
                    ball_pos_x.append(ball.pos[0])
                    ball_pos_y.append(ball.pos[1])

            if len(frame.camera_robots_blue) > 0:
                for robot in frame.camera_robots_blue:
                    camera_bot_pos_x.append(robot.pos[0])
                    camera_bot_pos_y.append(robot.pos[1])

            if len(frame.camera_robots_yellow) > 0:
                for robot in frame.camera_robots_yellow:
                    camera_bot_pos_x.append(robot.pos[0])
                    camera_bot_pos_y.append(robot.pos[1])


        self.camera_ball_line.set_xdata(ball_pos_x)
        self.camera_ball_line.set_ydata(ball_pos_y)

        if (self.world_ball is not None):
            self.world_ball_line.set_xdata(self.world_ball.pos[0])
            self.world_ball_line.set_ydata(self.world_ball.pos[1])
            self.world_ball_vel_x.append(self.world_ball.vel[0])
            self.world_ball_vel_y.append(self.world_ball.vel[1])
            self.world_ball_pos_cov.append(self.world_ball.pos_cov)
            self.world_ball_vel_cov.append(self.world_ball.vel_cov)
            self.time.append(self.time[len(self.time)-1] + 1)

        if len(self.time) > 50:
            l = len(self.time) - 50
            self.world_ball_vel_x = self.world_ball_vel_x[l:]
            self.world_ball_vel_y = self.world_ball_vel_y[l:]
            self.world_ball_pos_cov = self.world_ball_pos_cov[l:]
            self.world_ball_vel_cov = self.world_ball_vel_cov[l:]
            self.time = self.time[l:]

        for world_robot in self.world_robots_blue:
            if world_robot is not None:
                world_robot_blue_pos_x.append(world_robot.pos[0])
                world_robot_blue_pos_y.append(world_robot.pos[1])

        for world_robot in self.world_robots_yellow:
            if world_robot is not None:
                world_robot_yellow_pos_x.append(world_robot.pos[0])
                world_robot_yellow_pos_y.append(world_robot.pos[1])

        self.camera_robot_line.set_xdata(camera_bot_pos_x)
        self.camera_robot_line.set_ydata(camera_bot_pos_y)

        self.world_robot_blue_line.set_xdata(world_robot_blue_pos_x)
        self.world_robot_blue_line.set_ydata(world_robot_blue_pos_y)

        self.world_robot_yellow_line.set_xdata(world_robot_yellow_pos_x)
        self.world_robot_yellow_line.set_ydata(world_robot_yellow_pos_y)

        self.ax.set_title(str(self.time[len(self.time)-1]*util.config.dt))
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

        self.ball_vel_x_line.set_xdata(self.time)
        self.ball_vel_x_line.set_ydata(self.world_ball_vel_x)
        self.ball_vel_y_line.set_xdata(self.time)
        self.ball_vel_y_line.set_ydata(self.world_ball_vel_y)

        self.ax2.axis([min(self.time), min(self.time) + 50,
                       -6.5, 6.5])

        self.figure2.canvas.draw()
        self.figure2.canvas.flush_events()

        self.ball_pos_cov_line.set_xdata(self.time)
        self.ball_pos_cov_line.set_ydata(self.world_ball_pos_cov)
        self.ball_vel_cov_line.set_xdata(self.time)
        self.ball_vel_cov_line.set_ydata(self.world_ball_vel_cov)

        max_val = max(max(self.world_ball_pos_cov), max(self.world_ball_vel_cov))
        self.ax3.axis([min(self.time), min(self.time) + 50,
                       0, max(max_val, 10)])

        self.figure3.canvas.draw()
        self.figure3.canvas.flush_events()
