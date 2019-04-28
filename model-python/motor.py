"""Contents relate to motor modelling."""


class Motor:
    """Represents a motor."""

    def __init__(self, resistance, torque_constant,
                 speed_constant, rotor_inertia):
        """
        Create a new motor.

        params:
         resistance: motor terminal resistance, ohm
         torque_constant: motor torque constant, N m/A
         speed_constant: motor speed constant, V/(rad/s)
         rotor_inertia: rotor moment of inertia, kg m^2
        """
        self.resistance = resistance
        self.torque_constant = torque_constant
        self.speed_constant = speed_constant
        self.rotor_inertia = rotor_inertia

    def forward_dynamics(self, velocity, voltage):
        """
        Calculate motor torque.

        params:
         voltage: terminal voltage, V
         velocity: motor velocity, rad/s
        returns:
         torque, Nm
        """
        # T = Kt/R(V-Kv*v)
        emf = -self.speed_constant * velocity
        sum_voltage = voltage + emf
        return self.torque_constant * sum_voltage / self.resistance

    def inverse_dynamics(self, velocity, torque):
        """
        Calculate voltage required to apply given torque.

        params:
         velocity: motor velocity, rad/s
         torque: desired torque, Nm
        """
        # V = T*R/Kt + Kv*v
        emf = -self.speed_constant * velocity
        sum_voltage = torque * self.resistance / self.torque_constant
        return sum_voltage - emf

    def inertia(self):
        """Get the rotor inertia of this motor."""
        return self.rotor_inertia


class Gearbox:
    """Represents a gearbox."""

    def __init__(self, gear_ratio, gear_inertia):
        """
        Create a new gearbox.

        params:
         gear_ratio: the ratio of the gearbox, (output speed)/(input speed)
            e.g. a ratio of 3 means that the output shaft will have a third
            the torque of the original motor but three times the speed.
        """
        self.gear_ratio = gear_ratio
        self.gear_inertia = gear_inertia

    def output_velocity(self, input_velocity):
        """
        Get the speed of the output shaft given the speed of the input shaft.

        params:
         input_velocity: the input shaft velocity in m/s
        """
        return self.gear_ratio * input_velocity

    def input_velocity(self, output_velocity):
        """
        Get the speed of the output shaft given the speed of the input shaft.

        params:
         input_velocity: the input shaft velocity in m/s
        """
        return output_velocity / self.gear_ratio

    def output_torque(self, input_torque):
        """
        Get the torque of the output shaft given the torque of the input shaft.

        params:
         input_torque: the input shaft torque in Nm
        """
        return input_torque / self.gear_ratio

    def input_torque(self, output_torque):
        """
        Get the torque of the input shaft given the torque of the output shaft.

        params:
         output_torque: the output shaft torque in Nm
        """
        return output_torque * self.gear_ratio

    def inertia(self):
        """
        Get the inertia at the output shaft.
        """
        return self.gear_inertia


class GearMotor:
    """Represents a motor with attached gearbox."""

    def __init__(self, motor, gearbox):
        """
        Create a new gear motor.

        params:
         motor: the motor to use
         gearbox: the gearbox to use
        """
        self.motor = motor
        self.gearbox = gearbox

    def forward_dynamics(self, velocity, voltage):
        """
        Calculate torque on the output shaft.

         voltage: terminal voltage, V
         velocity: motor velocity on the output shaft, rad/s
        """
        input_velocity = self.gearbox.input_velocity(velocity)
        input_torque = self.motor.forward_dynamics(input_velocity, voltage)
        return self.gearbox.output_torque(input_torque)

    def inverse_dynamics(self, velocity, torque):
        """
        Calculate voltage required to apply given torque on the output shaft.

        params:
         velocity: output shaft velocity, rad/s
         torque: desired output torque, Nm
        """
        input_vel = self.gearbox.input_velocity(velocity)
        input_torque = self.gearbox.input_torque(torque)
        voltage = self.motor.inverse_dynamics(input_vel, input_torque)
        return voltage

    def inertia(self):
        """Get the effective inertia at the output shaft."""
        motor_inertia = self.motor.inertia() / self.gearbox.gear_ratio ** 2
        return self.gearbox.inertia() + motor_inertia

def main():
    motor = Motor(
        resistance=1.03,
        torque_constant=0.0335,
        speed_constant=0.0335,
        rotor_inertia=135*1e-3*(1e-2**2))
    gearbox = Gearbox(gear_ratio=20.0 / 60.0, gear_inertia=0)
    gear_motor = GearMotor(motor, gearbox)
    print(gear_motor.forward_dynamics(285 * 6.28 / 60.0, 3))
    print(gear_motor.forward_dynamics(30, gear_motor.inverse_dynamics(30, 0.17)))

if __name__ == '__main__':
    main()
