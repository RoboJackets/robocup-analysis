from camera.camera import Camera
import util.config

class World:
    def __init__(self):
        # Create list of cameras
        self.cameras = [None for x in range(0, util.config.max_num_cameras)]

        # Create list world robots / ball
        self.world_ball = []
        self.world_robots_blue = [[] for x in range(0, util.config.max_num_robots_per_team)]
        self.world_robots_yellow = [[] for x in range(0, util.config.max_num_robots_per_team)]

    # Function to take camera data from ssl_vision and forward it to all cameras
    # Merge the resulting kalman ball/robots 

    def update_with_camera_frame(self, frame_list):
        #calculate_ball_bounce()

        # Get list of camera frames to update
        # Update balls and robots corresponding to each camera frame
        # Update other stuff when we have no frames
        updated_camera_ids = []
        for frame in frame_list:
            camera_id = frame.camera_id

            if self.cameras[camera_id] is None:
                self.cameras[camera_id] = Camera(camera_id)

            self.cameras[camera_id].update_camera_balls(frame.camera_balls)
            self.cameras[camera_id].update_camera_robots(frame.kalman_robots_blue, 
                                                         frame.kalman_robots_yellow)

            updated_camera_ids.append(camera_id)

        for i in range(0, util.config.max_num_cameras):
            if (self.cameras[i] is not None and
                i not in updated_camera_ids):
                self.cameras[i].update_camera_without_data()

        #update_world_objects()
                

    def update_without_camera_frame(self):
        #calculate_ball_bounce()
        
        for camera in self.cameras:
            if camera is not None:
                camera.update_without_camera_frame()

        #update_world_objects()

    def calculate_ball_bounce(self):
        pass

    def update_world_objects(self):
        pass