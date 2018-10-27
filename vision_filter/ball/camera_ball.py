# Raw camera ball info from the cameras (For a single camera)
# Will also be the location of ball sense ball positions

class CameraBall:
    def __init__(self, time_captured, x, y):
        self.time_captured = time_captured
        self.pos = [x, y]