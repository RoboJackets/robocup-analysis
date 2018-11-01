# General object containing information for a kick

class KickDetection():
    def __init__(self, kick_time, measurement_list, kicking_robot):
        self.kick_time = kick_time
        self.measurement_list = measurement_list
        self.kicking_robot = kicking_robot