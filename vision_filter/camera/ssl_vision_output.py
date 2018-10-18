import camera.frame
import ball.camera_ball
import robot.camera_robot

def convert_frame(ssl_frame):
    frame_number = ssl_frame.detection.frame_number
    t_capture = ssl_frame.detection.t_capture
    t_sent = ssl_frame.detection.t_sent
    camera_id = ssl_frame.detection.camera_id
    ball_list = []
    robot_blue_list = []
    robot_yellow_list = []

    for frame_ball in ssl_frame.detection.balls:
        confidence = frame_ball.confidence
        x = frame_ball.x / 1000
        y = frame_ball.y / 1000

        camera_ball = ball.camera_ball.CameraBall(t_capture, confidence, x, y)
        ball_list.append(camera_ball)

    for frame_robot in ssl_frame.detection.robots_blue:
        confidence = frame_robot.confidence
        x = frame_robot.x / 1000
        y = frame_robot.y / 1000
        orientation = frame_robot.orientation
        robot_id = frame_robot.robot_id

        camera_robot = robot.camera_robot.CameraRobot(t_capture, confidence, x, y, orientation, robot_id)
        robot_blue_list.append(camera_robot)

    for frame_robot in ssl_frame.detection.robots_yellow:
        confidence = frame_robot.confidence
        x = frame_robot.x / 1000
        y = frame_robot.y / 1000
        orientation = frame_robot.orientation
        robot_id = frame_robot.robot_id

        camera_robot = robot.camera_robot.CameraRobot(t_capture, confidence, x, y, orientation, robot_id)
        robot_yellow_list.append(camera_robot)

    frame = camera.frame.Frame(frame_number, t_capture, t_sent, camera_id,
                               ball_list, robot_blue_list, robot_yellow_list)

    return frame