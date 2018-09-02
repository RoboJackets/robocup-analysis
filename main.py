from camera.camera import Camera
from camera.fake_ssl_vision_output import FakeSSLVisionOutput

from camera.world import World

import matplotlib.pyplot as plt
import time

import util.config

# Look at 1/dt hz
# Check for camera frame
# If we get one, update all the stuff for that camera
# If not, just update without data

vis = FakeSSLVisionOutput()
world = World()

for i in range(500):
    frames = vis.get_frames(i*util.config.dt*2*3.14 + .1)
    world.update_with_camera_frame(frames)
    
    time.sleep(util.config.dt)