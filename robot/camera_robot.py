# Kalman filter for one robot

class CameraRobot:
    def __init__(self, time_captured, confidence, x, y, bot_id):
        self.time_captured = time_captured
        self.confidence = confidence
        self.x = x
        self.y = y
        self.bot_id = bot_id