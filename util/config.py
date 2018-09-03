sample_rate = 100
dt = 1/sample_rate
camera_rate = 60

robot_init_covariance = 0.1
robot_process_noise = 0.1
robot_observation_noise = 0.1

ball_init_covariance = 100
ball_process_error = 0.1
ball_observation_noise = 2.0

max_num_robots_per_team = 20
max_num_cameras = 10

use_multi_hypothesis = False
multi_hypothesis_radius_cutoff = .1
single_kalman_radius_cutoff = .1

health_init = 2
health_dec = 1
health_inc = 2
health_max = 20
health_bad = 1
health_min = 1

field_width = 10
field_length = 20
num_cameras_width = 2
num_cameras_length = 4
camera_percent_overlap = 0

ball_merger_power = 1.5 # Should be between 1 and 2, higher number results in more jitter