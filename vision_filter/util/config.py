# Program flow controls
use_fake_camera = False

# Basic Filter settings
sample_rate = 100
dt = 1/sample_rate

# Static settings for size
robot_radius = .07 # Not really correct, but the mouth causes problems since it's not implemented
ball_radius = .02134

# Collision settings
# How what percent of the ball velocity is left after collision to each
ball_robot_circle_angle_dampen_factor = 0.0 # How much to change the angle. Higher means less like ideal
ball_robot_circle_vel_dampen_factor   = 0.7 # What percent of vel is left after collision
ball_robot_mouth_angle_dampen_factor  = 0.0
ball_robot_mouth_vel_dampen_factor    = 1.0

# Kalman filter settings
ball_init_covariance = 100
ball_process_error = 0.1
ball_observation_noise = 2.0

robot_init_covariance = 100
robot_process_noise = 0.1
robot_observation_noise = 2.0

# Multi Hypothesis
use_multi_hypothesis = True
multi_hypothesis_radius_cutoff = 1

ball_merger_power = 1.5 # Should be between 1 and 2, higher number results in more jitter
robot_merger_power = 1.5 # Should be between 1 and 2, higher number results in more jitter

# Kalman filter health
health_init = 2 # What health it starts out with
health_dec = 1  # Every prediction (without a observation) we decrement the health by this amount
health_inc = 2  # Every prediction with observation we increment the health
health_max = 20 # Max health limit
health_min = 1  # Minimum health limit (must be > 0) to keep the math nice

# Kalman filter removal requirements
ball_max_time_outside_vision = .2
robot_max_time_outside_vision = 2.0

# Kick detection settings
slow_kick_detector_history_length = 5

# Various field information
field_width = 12.0
field_length = 9.0
num_cameras_width = 2
num_cameras_length = 8
camera_percent_overlap = 0

# Limits on array sizes
max_num_robots_per_team = 20
max_num_cameras = 20

# Fake camera settings
camera_noise = 0.01
sim_ball_speed = 6.0