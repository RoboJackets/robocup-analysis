sample_rate = 100
dt = 1/sample_rate

robot_init_covariance = 0.1
robot_process_noise = 0.1
robot_observation_noise = 0.1

ball_init_covariance = 0.1
ball_process_noise = 0.1
ball_observation_noise = 0.1

max_num_robots_per_team = 20
max_num_cameras = 10

use_multi_hypothesis = False
multi_hypothesis_radius_cutoff = .1
single_kalman_radius_cutoff = .1

health_init = 10
health_dec = 1
health_inc = 2
health_max = 100