import kick.kick_detector
import kick.kick_detection
import util.config
import numpy as np
import math

# Take last 3 ball applied to each kalman filter
# Check direction change
#       speed change
#       distance to closest robot
# If so, get kicking robot
# Get list of balls since kick time
# Create kick event
# check velocity jump between 1->2 and 2->3
# Trigger on angle + vel jump
# Get list of all merged balls we have updated since kick time
# Kick time is middle ball time

class FastKickDetector(kick.kick_detector.KickDetector):
    def __init__(self):
        # List of tuples that include (time, world ball, world robots)
        self.state_history = []

    def add_record(self, time, world_ball, world_robot_list):
        # Get 3 camera_balls
        # detect kick
        # Figure out which robot kicked
        # Grab all balls since kick
        # Return kick event

        # Append current world ball and keep within a certain length
        self.state_history.append((time, world_ball))
        if len(self.state_history) > util.config.fast_kick_detector_history_length:
            self.state_history.pop(0)

        # Make sure we actually have at least 3 measurements
        if len(self.state_history) < util.config.fast_kick_detector_history_length:
            return None

        # See if there was a kick based on the N frames
        if not self.detect_kick():
            return None

        # Get closest robot to second ball since it was the last entry before the kick
        closest_robot = self.get_closest_robot(world_robot_list)

        # time of kick
        kick_time = self.state_history[math.ceil(len(self.state_history) / 2)][0]

        # List of ball measurements since (and including) kick instance
        measurement_list = self.state_history[math.ceil(len(self.state_history) / 2):]

        # Returns a kick object that we can use as an initilation of the kick trajectory solvers
        return kick.kick_detection.KickDetection(kick_time, measurement_list, closest_robot)

    def reset(self):
        self.state_history = []

    def detect_kick(self):
        # 3 balls, accelerations
        # return true on large velocity jump
        # return true on large vel and direction jump
        e = len(self.state_history) - 1

        dp1 = np.subtract(self.state_history[1][1].pos, self.state_history[0][1].pos)
        dp2 = np.subtract(self.state_history[e][1].pos, self.state_history[e - 1][1].pos)

        v1 = np.multiply(dp1, 1/util.config.dt)
        v2 = np.multiply(dp2, 1/util.config.dt)

        dv = np.subtract(v2, v1)

        a = np.multiply(dv, 1/util.config.dt)

        a_mag = np.linalg.norm(a)

        return a_mag > util.config.fast_kick_velocity_change_trigger

    def get_closest_robot(self, world_robot_list):
        mid_ball_pos = self.state_history[math.ceil(len(self.state_history) / 2)][1].pos

        min_robot = None
        min_dist = float('inf')

        for r in world_robot_list:
            if r is not None:
                bot_pos = r.pos
                bot_pos.pop(2) # Remove orientation to make things easy

                dist = np.linalg.norm(np.subtract(mid_ball_pos, bot_pos))

                if (dist < min_dist):
                    min_robot = r
                    min_dist = dist


        return min_robot
