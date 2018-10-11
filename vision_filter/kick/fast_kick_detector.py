import kick.kick_detector

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

    def add_record(self, world_ball, world_robot_list):
        # Get 3 camera_balls
        # detect kick
        # Figure out which robot kicked
        # Grab all balls since kick
        # Return kick event
        pass

    def reset(self):
        pass

    def detect_kick(self):
        # 3 balls, accelerations
        # return true on large velocity jump
        # return true on large vel and direction jump
        pass