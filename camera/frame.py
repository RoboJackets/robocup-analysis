
class Frame:
    def __init__(self, 
                 frame_number, 
                 t_capture, t_sent, 
                 camera_id, 
                 camera_balls, 
                 camera_robots_yellow, camera_robots_blue):
        
        self.frame_number = frame_number
        self.t_capture = t_capture
        self.t_sent = t_sent
        self.camera_id = camera_id
        self.camera_balls = camera_balls
        self.camera_robots_yellow = camera_robots_yellow
        self.camera_robots_blue = camera_robots_blue
