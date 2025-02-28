#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# See the notes for the other physics sample
#

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain, FourMotorDrivetrain
from pyfrc.physics.units import units

import typing

if typing.TYPE_CHECKING:
    from robot import robot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"): # type: ignore
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """
        self.robot = robot
        self.physics_controller = physics_controller

        # Initialize Motors and sensors
        self.gyro = wpilib.simulation.AnalogGyroSim(robot.poseEstimator.gyro)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Updates simulation parameters
        :param now: Current time
        :param tm_diff: Time difference since last update
        """

        # Retrieve the motor values from the drivetrain

        # Apply movement to simulation
        transform = self.physics_controller.move_robot(chassis_speeds)

        # Update gyro simulation (negative to match FRC convention)
        self.gyro.setAngle(-transform.rotation().degrees())