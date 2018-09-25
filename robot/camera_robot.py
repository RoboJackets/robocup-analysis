# Kalman filter for one robot

class CameraRobot:
    def __init__(self, time_captured, confidence, x, y, theta, bot_id):
        self.time_captured = time_captured
        self.confidence = confidence
        self.pos = [x, y]
        self.theta = theta
        self.bot_id = bot_id