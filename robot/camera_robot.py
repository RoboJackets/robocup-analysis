# Kalman filter for one robot

class CameraRobot:
    def __init__(self, confidence, x, y, orientation, bot_id):
        self.confidence = confidence
        self.x = x
        self.y = y
        self.orientation = orientation
        self.bot_id = bot_id