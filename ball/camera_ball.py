# Raw camera ball info from the cameras (For a single camera)
# Will also be the location of ball sense ball positions

class CameraBall:
    def __init__(self, time_captured, confidence, x, y):
        self.time_captured = time_captured
        self.confidence = confidence
        self.x = x
        self.y = y