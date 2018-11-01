import kick.kick_detector
import util.config

# Get kicking bot based on direction facing, min ball vel, infront, distance (was near, now away), increasing distance

# Collect all new balls along that are detected for each of these filters
# Try to fit various things to it

class SlowKickDetector(kick.kick_detector.KickDetector):
    def __init__(self):
        # Keeps list of the history over the last X frames
        # List of tuples that include (time, world ball, world robots)
        self.state_history = []

        self.last_kick_time = 0

    def add_record(self, time, world_ball, world_robot_list):
        self.state_history.append((time, world_ball, world_robot_list))

        if (len(self.state_history) > util.config.slow_kick_detector_history_length):
            self.state_history.pop(0)

        if (len(self.state_history) == util.config.slow_kick_detector_history_length):
            return self.process()

        return None

    def reset(self):
        print("ERROR::KickDetector::VirtualMethod")

    def process(self):
        # List of times from each state
        time_list       = []
        # List of balls from each state
        ball_list       = []
        # List of robot id's, where each item is a list of them each state
        robot_list_list = [None] * 2*util.config.max_num_robots_per_team

        for state in self.state_history:
            time_list.append(state[0])
            ball_list.append(state[1])
            for idx, robot in enumerate(state[2]):
                if robot is not None:
                    robot_list_list[idx].append(robot)

        # Remove any robots that aren't in all of the state history
        robot_list_list = list(filter(
            lambda robot_list:
                len(robot_list) < util.config.slow_kick_detector_history_length,
            robot_list_list))

        # Remove robots not near ball?

        # For each bot list
        # Test kick validators on bot and ball list
        # Select list as kickerBot

        # Create KickEvent
        # Try backtrack to find kick point