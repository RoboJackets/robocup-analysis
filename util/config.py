sample_rate = 100
dt = 1/sample_rate
camera_rate = 60

robot_init_covariance = 100
robot_process_noise = 0.1
robot_observation_noise = 2.0

robot_radius = .1
ball_radius = .05

# How what percent of the ball velocity is left after collision to each
ball_robot_circle_angle_dampen_factor = 0.5
ball_robot_circle_vel_dampen_factor   = 1.0
ball_robot_mouth_angle_dampen_factor  = 0.0
ball_robot_mouth_vel_dampen_factor    = 1.0

ball_init_covariance = 100
ball_process_error = 0.1
ball_observation_noise = 2.0

max_num_robots_per_team = 20
max_num_cameras = 10

use_multi_hypothesis = False # True
multi_hypothesis_radius_cutoff = 1

health_init = 2 # What health it starts out with
health_dec = 1  # Every prediction (without a observation) we decrement the health by this amount
health_inc = 2  # Every prediction with observation we increment the health
health_max = 20 # Max health limit
health_min = 1  # Minimum health limit (must be > 0) to keep the math nice
health_bad = 1  # Removes kalman_ball when health less then equal to this

field_width = 10
field_length = 20
num_cameras_width = 2
num_cameras_length = 4
camera_percent_overlap = 0

ball_merger_power = 1.5 # Should be between 1 and 2, higher number results in more jitter
robot_merger_power = 1.5 # Should be between 1 and 2, higher number results in more jitter

camera_noise = 0.01
sim_ball_speed = 6.0
