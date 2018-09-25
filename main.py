from sys import platform
if platform == "darwin":
    # MACOS - use a different backend so the animations work
    # note, this import must be before any other matploblib import
    import matplotlib
    matplotlib.use('TkAgg')

from camera.camera import Camera
from camera.fake_ssl_vision_output import FakeSSLVisionOutput

from camera.world import World

import matplotlib.pyplot as plt
import time
import datetime

import util.config

# Look at 1/dt hz
# Check for camera frame
# If we get one, update all the stuff for that camera
# If not, just update without data

vis = FakeSSLVisionOutput()
world = World()

for i in range(500):
    start = datetime.datetime.now()

    frames = vis.get_frames(i*util.config.dt)
    world.update_with_camera_frame(frames)

    end = datetime.datetime.now()
    delta = int((end - start).total_seconds()*1000) # ms

    if (delta < util.config.dt):
        time.sleep(delta - util.config.dt)

    if (i % 100 == 0):
        print(1/delta*1000)