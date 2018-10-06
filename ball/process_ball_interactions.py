import util.config
import numpy as np
import math
# Try to predict bounces off of other robots
# Just throwing into it's own file to make things easier and more modular

# Needs kalman ball and world robot estimations
def calculate_bounce(estimated_ball, estimated_robots):
    # Check intersection of robot circle and ball for next time step
    # Calculate line circle intersection
    # Using velocity, take first edge intersection (closest distance of cur_pos - vel)
    # If intersects around the mouth
    # Do flat line intersections

    for robot in estimated_robots:
        if robot is None:
            continue

        ball_intersects = ball_in_robot(estimated_ball, robot)

        if not ball_intersects:
            continue

        intersect_points = possible_ball_intersection_pts(estimated_ball, robot)

        # No intersection (should be captured by the intersection above already)
        if len(intersect_points) == 0:
            continue

        # Line is tangent to the robot
        # Assume no interaction (Good assumption?)
        if len(intersect_points) == 1:
            continue

        # Passes through two points
        if len(intersect_points) == 2:

            #                        _____
            #                       /     \
            #                      | Robot |
            #                       \_____/
            #                          B
            #                         /|\
            #                        / | \
            #                       /  |  \
            #                      /   |   \
            #                     /    D    \
            #                    A           C
            # Ball moves from A->B
            # Bounces off the robot
            # Moves from B->C
            # Line B<->D is the line of reflection
            #
            #
            # Line B<->D goes from the center of the robot through the point of intersection between the robot and the ball
            # Since the robot is round (with a flat mouth), any time it hits the robot, we can assume that the it will reflect as if
            #   it hit a flat surface

            # We want to make sure that the ball is never inside the robot when doing this math, so we go back in time
            # This doesn't affect the results since we are just doing vectors
            ball_pos_safe_point = np.subtract(estimated_ball.pos(), np.multiply(estimated_ball.vel(), 1/np.linalg.norm(estimated_ball.vel())))

            # Find the closest point
            # AKA: the first point the ball will hit off the robot
            # May actually be slightly off because we add the ball radius to the circle thing
            # TODO: Account of the mouth
            # This is super easy, just check intersection with the mouth line
            # If the first point in the circle shell intersect is in the mouth angle range
            # Use the line intersect point instead
            # If there is no line intersect point (within that angle range)
            # then the ball is moving across the mouth of the robot
            closest = intersect_points[0]
            if dist_between_pts(ball_pos_safe_point, closest) > dist_between_pts(ball_pos_safe_point, intersect_points[1]):
                closest = intersect_points[1]

            #                          R
            #                        _____
            #                          B
            #                         /|\
            #                        / | \
            #                       /  |  \
            #                      /   |   \
            #                     /    |    \
            #                    A-----D-----C

            # B->A
            intersect_point_ball_vector  = np.subtract(ball_pos_safe_point, closest)
            # B->D (Officially R->B, but B->D makes more sense visually)
            robot_intersect_point_vector = np.subtract(closest, robot.pos[0:2])
            robot_intersect_point_mag    = np.linalg.norm(robot_intersect_point_vector)
            robot_intersect_point_unit   = np.multiply(robot_intersect_point_vector, 1/robot_intersect_point_mag)

            # Project the B->A vector onto B->D
            # This is so we can get D->A and D->C later on
            projection_mag = np.dot(intersect_point_ball_vector, robot_intersect_point_vector) / robot_intersect_point_mag
            projection = np.multiply(robot_intersect_point_unit, projection_mag)

            # A->D, which is the same as D->C
            difference = np.subtract(projection, intersect_point_ball_vector)

            # B->C which is where a perfect ball will go with no elastic collisions
            intersect_point_reflection_vector = np.add(difference, projection)
            intersect_point_reflection_mag    = 1/np.linalg.norm(intersect_point_reflection_vector)
            intersect_point_reflection_unit   = np.multiply(intersect_point_reflection_vector, intersect_point_reflection_mag)

            # Scale the overal magnitude of the velocity to account for the bounce taking energy
            speed_scale_matrix = [[util.config.ball_robot_circle_vel_dampen_factor, 0],
                                  [0, util.config.ball_robot_circle_vel_dampen_factor]]

            #                   C------D
            #                    \     |
            #                F    \    |
            #                  \   \   |
            #                    \  \  |
            #                      \ \ |
            #                        \\|              | y+
            #                          B              |
            #                                         |____ x+
            #
            # Note: Letters correspond to ones above
            #
            # Convert to this coordinate system where the origin is at B
            # This allows us to more easily change the angle of the B->C reflection vector to account for the angle changes
            #    on the reflection
            # We are trying to increase the angle CBD more when angle CBD is large
            # When angle CBD is 0, we want to keep the same angle

            # Get transform between a world coordinate system and the one just above
            angle_scale_transform = [[projection[1], -projection[0]],
                                     [projection[0],  projection[1]]]
            angle_scale_transform = np.multiply(angle_scale_transform, 1/np.linalg.norm(angle_scale_transform, 2))

            # Inverse of rotation matrix is just the transpose
            inv_angle_scale_transform = np.transpose(angle_scale_transform)

            # B->F vectpr
            reflection_transformed_unit = np.dot(angle_scale_transform, intersect_point_reflection_unit)

            # Angle CBD
            half_total_reflection_angle = math.acos(min(intersect_point_reflection_unit.dot(robot_intersect_point_unit), 1))

            # Find the direction to rotate towards the x axis based on the sign of the X value of the B->F vector
            additional_rotate_angle = -sign(reflection_transformed_unit[0]) * half_total_reflection_angle

            # We dont want any extra rotation when angle CBD is 0 degrees or 90 degrees
            # Just to simplify implementation, I'm going to do a triangle
            #
            # df*45  -              /  \
            #                    /        \
            #                 /              \
            #  0     -     /                    \
            #
            #             |          |           |
            #            0 deg    45 deg       90 deg
            #
            # df is angle dampen factor
            # y axis represents max angle dampen in terms of degrees
            # x axis is the angle CBD
            additional_rotate_angle = min([additional_rotate_angle, math.pi/2 - additional_rotate_angle]) * util.config.ball_robot_circle_angle_dampen_factor

            # Rotate the vector by this much more towards the X axis
            t = additional_rotate_angle # Just to get this shorter
            angle_scale_matrix = [[math.cos(t), -math.sin(t)],
                                  [math.sin(t),  math.cos(t)]]

            # Multiply everything together
            full_transform = np.dot(inv_angle_scale_transform, angle_scale_matrix)
            full_transform = np.dot(full_transform, speed_scale_matrix)
            full_transform = np.dot(full_transform, angle_scale_transform)

            # Get final velocity direction and scale by multiplying the intersect point->reflection vector by the dampen transform matrices
            # and multiplying it by the intial velocity of the ball
            final_ball_vel = full_transform.dot(intersect_point_reflection_unit)
            final_ball_vel = np.multiply(final_ball_vel, np.linalg.norm(estimated_ball.vel()))

            return final_ball_vel

    # No intersections found
    return None

# Ball within the robot radius (Misses mouth stuff, but whatevs)
def ball_in_robot(estimated_ball, estimated_robot):
    next_ball_pos = np.add(estimated_ball.pos(), np.multiply(estimated_ball.vel(), util.config.dt))
    return dist_between_pts(next_ball_pos, estimated_robot.pos[0:2]) < util.config.robot_radius + util.config.ball_radius

# Simple dist between (x,y) points
def dist_between_pts(pos1, pos2):
    dx = pos2[0] - pos1[0]
    dy = pos2[1] - pos1[1]
    return math.sqrt(dx*dx + dy*dy)

# If the ball intersects a robot
# Find intersection points (1 or 2) between the ball line and robot
# Returns two points for that
def possible_ball_intersection_pts(estimated_ball, estimated_robot):
    # http://mathworld.wolfram.com/Circle-LineIntersection.html

    # x/y1 is the current ball in the robot centered coordinate frame
    # x/y2 is the next probable ball position in the robot centered coordinate frame
    # We just want a line in the direction of the ball motion
    x1 = estimated_ball.pos()[0] - estimated_robot.pos[0]
    y1 = estimated_ball.pos()[1] - estimated_robot.pos[1]
    x2 = estimated_ball.pos()[0] + estimated_ball.vel()[0] - estimated_robot.pos[0]
    y2 = estimated_ball.pos()[1] + estimated_ball.vel()[1] - estimated_robot.pos[1]

    dx = x2 - x1
    dy = y2 - y1
    dr = math.sqrt(dx*dx + dy*dy)
    D = x1*y2 - x2*y1 # Determinant
    r = util.config.robot_radius + util.config.ball_radius


    # Don't forget to move the coordinate system back into the normal world
    x1 = D*dy + sign(dy)*dx * math.sqrt(r*r*dr*dr - D*D)
    x1 /= dr*dr
    x1 += estimated_robot.pos[0]

    y1 = -D*dx + abs(dy) * math.sqrt(r*r*dr*dr - D*D)
    y1 /= dr*dr
    y1 += estimated_robot.pos[1]

    x2 = D*dy - sign(dy)*dx * math.sqrt(r*r*dr*dr - D*D)
    x2 /= dr*dr
    x2 += estimated_robot.pos[0]

    y2 = -D*dx - abs(dy) * math.sqrt(r*r*dr*dr - D*D)
    y2 /= dr*dr
    y2 += estimated_robot.pos[1]

    delta = r*r*dr*dr - D*D

    # No intersection
    if delta < 0:
        return []

    # Tangent
    elif delta == 0:
        return [[x1, y1]]

    # Two point intersection
    else: # detlta > 0
        return [[x1, y1], [x2, y2]]

def sign(val):
    if val >= 0:
        return 1
    else:
        return -1

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