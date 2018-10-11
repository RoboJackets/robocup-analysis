from sys import platform

if platform == "darwin":
    # MACOS - use a different backend so the animations work
    # note, this import must be before any other matploblib import
    import matplotlib
    matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import time
import datetime

# Camera imports
from camera.fake_ssl_vision_output import FakeSSLVisionOutput
import rc_ssl_logtools
import camera.ssl_vision_output

from camera.world import World
import util.config


# Initialize cameras stuff
if util.config.use_fake_camera:
    vis = FakeSSLVisionOutput()
    iteration_length = 100
else: # 5 - 10 # 20~ # 51
    log_frames = rc_ssl_logtools.log_frames("logs/2018-06-21_14-09_ZJUNlict-vs-CMμs.log.gz", 51, 60)
    iteration_length = len(log_frames)

for i in range(0,20):
    print(str(log_frames[i].detection.t_sent))

# Get start time
if util.config.use_fake_camera:
    current_time = 0
else:
    start_frame = camera.ssl_vision_output.convert_frame(log_frames[0])
    current_time = start_frame.t_capture

index = 0

world = World()

while index < iteration_length:
    start = datetime.datetime.now()

    current_time += util.config.dt # Time is in seconds

    # Get the frames from vision
    if util.config.use_fake_camera:
        # Fake camera data based just on the current time
        frames = vis.get_frames(current_time)
        index += 1
    else:
        # Grab frames from before the current step
        frames = []
        while True:
            frame = camera.ssl_vision_output.convert_frame(log_frames[index])

            if frame.t_capture < current_time:
                frames.append(frame)
                index += 1
            else:
                break

    # Update all our filters
    world.update_with_camera_frame(current_time, frames)

    # Wait to try and hit the loop times
    end = datetime.datetime.now()
    delta = int((end - start).total_seconds()*1000) # ms

    if (delta < util.config.dt):
        time.sleep(delta - util.config.dt)

    if (index % 100 == 0):
        print(1/delta*1000)