import util.config
import math
# Try to predict bounces off of other robots
# Just throwing into it's own file to make things easier and more modular

# Needs kalman ball and world robot estimations
def calculate_bounce(estimated_ball, estimated_robots):
    # Check intersection of robot circle and ball
    # Calculate line circle intersection
    # Using velocity, take first edge intersection (closest distance of cur_pos - vel)
    # If intersects around the mouth
    # Do flat line intersections

    # Using normal
    # Find angle of hit
    # Use config on angle and speed since this is elastic collision
    # Return / edit estimated ball
    pass

# Ball within the robot radius (Misses mouth stuff, but whatevs)
def ball_in_robot(estimated_ball, estimated_robot):
    return dist_between_pts(estimated_ball.x, estimated_ball.y,
                            estimated_ball.x, estimated_ball.y) < util.config.robot_radius

# Simple dist between (x,y) points
def dist_between_pts(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx*dx + dy*dy)

# If the ball intersects a robot
# Find intersection points (1 or 2) between the ball line and robot
# Returns two points for that
def possible_ball_intersection_pts(estimated_ball, estimated_robot):
    # Get two points
    # Do projection from circle center
    # Since it's less then radius
    # We can get the angles for the two intersection chords

    # Ball line to center robot
    # Project that line onto ball line
    # Check for edge (just return if they are almost radius)
    # acos(dist from proj to center / radius)
    # Rotate radius vector +- that angle
    # Return list of two points
    pass

def get_normal_of_intersection_point(intersect_point, estimated_robot):
    # Check the mouth or circle
    # If mouth, just do forward vector
    # If side, do center to intersect point
    pass

def calc_reflection(normal, estimated_ball):
    # Get angle between ball vel and normal
    # Soften angle based on config (percentage)
    # Soften vel based on config (percentage)
    pass