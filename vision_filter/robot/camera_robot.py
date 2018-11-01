# Kalman filter for one robot

class CameraRobot:
    def __init__(self, time_captured, x, y, theta, bot_id):
        self.time_captured = time_captured
        self.xypos = [x, y]
        self.theta = theta
        self.pos = [x, y, theta]
        self.bot_id = bot_id