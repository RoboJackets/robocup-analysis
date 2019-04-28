"""Robocup robot modelling."""

import numpy as np
import motor
import util


class Robot:
    """A robocup robot."""

    def __init__(self,
                 robot_mass,
                 robot_radius,
                 gear_motor,
                 robot_inertia,
                 wheel_radius,
                 wheel_inertia,
                 wheel_angles):
        """Create a new robot."""
        # wheel (angular) velocity = geom * body velocity
        # geom.T * wheel torque = body wrench
        self.geom = np.asmatrix(
            [[-np.sin(th), np.cos(th), robot_radius] for th in wheel_angles]
        ) / wheel_radius

        self.gear_motor = gear_motor
        self.robot_mass_mat = np.asmatrix([
            [robot_mass, 0, 0],
            [0, robot_mass, 0],
            [0, 0, robot_inertia],
        ])

        identity = np.asmatrix(np.eye(4))
        wheel_inertia = (wheel_inertia + gear_motor.inertia()) * identity
        wheel_mass_mat = self.geom.T * wheel_inertia * self.geom
        self.total_mass_mat = self.robot_mass_mat + wheel_mass_mat

        self.wheel_friction_mat = np.asmatrix(np.eye(3)) * 0

    def forward_dynamics_body(self, velocity, voltage):
        """
        Run the continuous model to get a robot acceleration twist.

        Calculations are run in the inertial frame that currently coincides
        with the body frame.

        params:
         velocity: body-space velocity vector [x; y; theta]
         voltage: wheel voltage vector [v1; v2; v3; v4]
        """
        wheel_velocity = self.geom * velocity
        torque = self.gear_motor.forward_dynamics(wheel_velocity, voltage)
        effort_body = self.geom.T * torque

        coriolis = np.asmatrix([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 0]
        ]) * velocity[2, 0]

        drag_matrix = self.wheel_friction_mat + coriolis * self.robot_mass_mat

        return np.linalg.inv(self.total_mass_mat) * (
            effort_body - drag_matrix * velocity)

    def inverse_dynamics_body(self, velocity, acceleration):
        """
        Find the voltage corresponding to a particular acceleration twist.

        params:
         velocity: body-space velocity vector [x; y; theta]
         acceleration: desired body-space acceleration vector [x; y; theta]
        """
        wheel_velocity = self.geom * velocity

        coriolis = np.asmatrix([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 0]
        ]) * velocity[2, 0]

        geom_inv = np.linalg.pinv(self.geom)

        drag_matrix = self.wheel_friction_mat + coriolis * self.robot_mass_mat
        net_torque = geom_inv.T * self.total_mass_mat * acceleration
        drag_torque = geom_inv.T * drag_matrix * geom_inv * wheel_velocity
        required_torque = net_torque + drag_torque

        result = self.gear_motor.inverse_dynamics(
            wheel_velocity, required_torque)
        return result

    def forward_dynamics_world(self, pose, velocity, voltage):
        """
        Perform the forward dynamics calculation in world space.

        params:
         pose: world-space pose vector [x; y; theta]
         velocity: world-space velocity vector [x; y; theta]
         voltage: wheel voltage vector [v1; v2; v3; v4]
        """
        coriolis = np.asmatrix([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 0]
        ]) * velocity[2, 0]

        rotation_matrix = util.rotation_matrix(pose[2, 0])
        velocity_body = rotation_matrix.T * velocity
        acceleration_body = self.forward_dynamics_body(velocity_body, voltage)
        return coriolis * velocity + rotation_matrix * acceleration_body

    def inverse_dynamics_world(self, pose, velocity, acceleration):
        """
        Perform the forward dynamics calculation in world space.

        params:
         pose: world-space pose vector [x; y; theta]
         velocity: world-space velocity vector [x; y; theta]
         voltage: wheel voltage vector [v1; v2; v3; v4]
        """
        coriolis = np.asmatrix([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 0]
        ]) * velocity[2, 0]

        rotation_matrix = util.rotation_matrix(pose[2, 0])
        velocity_body = rotation_matrix.T * velocity
        acceleration_body = rotation_matrix.T * acceleration - \
            coriolis * velocity_body
        return self.inverse_dynamics_body(velocity_body, acceleration_body)


def main():
    maxon_motor = motor.Motor(
        resistance=1.03,
        torque_constant=0.0335,
        speed_constant=0.0335,
        rotor_inertia=135*1e-3*(1e-2**2))
    gearbox = motor.Gearbox(gear_ratio=20.0 / 60.0, gear_inertia=0)
    gear_motor = motor.GearMotor(maxon_motor, gearbox)
    robot = Robot(
        robot_mass=6.5,
        robot_radius=0.085,
        gear_motor=gear_motor,
        robot_inertia=6.5*0.085*0.085*0.5,
        wheel_radius=0.029,
        wheel_inertia=2.4e-5,
        wheel_angles=np.deg2rad([45, 135, -135, -45]))
    velocity = np.asmatrix([-1.0, -2, 3]).T
    voltage = np.asmatrix([24.0, 24, 24, 24]).T
    accel = np.asmatrix([1, 2, 3]).T
    pose = np.asmatrix([1, 2, 1]).T
    print(
        robot.forward_dynamics_world(
            pose, velocity, robot.inverse_dynamics_world(
                pose, velocity, accel)))
    # print(
    #     robot.forward_dynamics_body(
    #         velocity, robot.inverse_dynamics_body(
    #             velocity, accel)))
    # print(
    #     robot.inverse_dynamics_world(
    #         pose, velocity, robot.forward_dynamics_world(
    #             pose, velocity, voltage)))
    # print(robot.forward_dynamics_world(
    #         np.asmatrix([0, 0, np.deg2rad(0)]).T,
    #         np.asmatrix([0, 0, 1]).T,
    #         np.asmatrix([5, 5, -5, -5]).T))
    # for _ in range(0):
    #     velocity += 0.01 * robot.forward_dynamics(velocity, voltage)
    #     print(velocity.T)
    #     print('-----')

if __name__ == '__main__':
    main()
