"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2023
Code for robot ""
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

# This is to help vscode
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot  # type: ignore

import wpilib
import wpilib.interfaces
import subsystems.leds
from commands2 import ParallelCommandGroup, Subsystem, WaitCommand

# from commands2.button import Button
from wpilibextra.customcontroller.custom_button import CustomButton as Button
from wpilib import DriverStation
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import math

from wpimath.estimator import SwerveDrive4PoseEstimator

import const


# Controls
from wpilibextra.customcontroller import XboxCommandController

###  IMPORTS ###


class OI:
    """
    Operator Input - This class ties together controls and commands.
    """

    def __init__(self, robot: "Robot"):
        self.robot = robot

        # Controllers
        self.driver1 = XboxCommandController(0)
        self.driver2 = XboxCommandController(1)

        # self.driver1.LEFT_JOY_Y.setInverted(True)

        self.driver1.LEFT_JOY_X.setDeadzone(0.02)
        self.driver1.LEFT_JOY_Y.setDeadzone(0.02)
        self.driver1.RIGHT_JOY_X.setDeadzone(0.1)
        self.driver1.RIGHT_JOY_Y.setDeadzone(0.1)

        self.driver2.LEFT_JOY_X.setDeadzone(0.02)
        self.driver2.LEFT_JOY_Y.setDeadzone(0.02)
        self.driver2.RIGHT_JOY_X.setDeadzone(0.1)
        self.driver2.RIGHT_JOY_Y.setDeadzone(0.1)

        # self.driver1.LEFT_JOY_X.setDeadzone(0.000)
        # self.driver1.LEFT_JOY_Y.setDeadzone(0.000)
        # self.driver1.RIGHT_JOY_X.setDeadzone(0.00)
        # self.driver1.RIGHT_JOY_Y.setDeadzone(0.00)

        ### Driving ###
        self.cardinal = 0
        self.cardinal_directing = False
        self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()

        self.rumble_button = Button(lambda: self.robot.has_note)
        self.can_crash = False

        self.looking_for_algae = False

        @self.rumble_button.whenPressed
        def _():
            timer = Timer()
            timer.start()
            self.driver2.setRumble(1)
            self.driver1.setRumble(1)
            while not timer.hasElapsed(0.5):
                yield
            self.driver2.setRumble(0)
            self.driver1.setRumble(0)

        @self.robot.drivetrain.setDefaultCommand
        def _():
            while True:
                yield

                def square(x):
                    return abs(x) * x

                forward_back = -square(self.driver1.LEFT_JOY_Y())
                left_right = -square(self.driver1.LEFT_JOY_X())
                if not self.driver1.RIGHT_TRIGGER_AS_BUTTON():  # boost
                    forward_back *= 0.8
                    left_right *= 0.8
                
                rotate = -self.driver1.RIGHT_JOY_X() 
                self.robot_oriented_angle += rotate * 15.0  # Type: ignore

                if self.looking_for_algae:
                    drive_magnitude = math.hypot(forward_back, left_right)
                    angle_result = self.robot.limelight.angle_to_nearest_algae

                    if angle_result is not None and (
                        rotate == 0
                    ):  # If the limelight sees note
                        self.robot.leds.set_mode(robot.leds.MODE_LOCKED_ON)
                        angle_to_nearest_algae = (
                            self.robot.poseEstimator.getYaw().degrees()
                            - angle_result * 0.65
                        ) % 360

                        direction_angle = (
                            math.atan2(forward_back, -left_right) * (180 / math.pi)
                            - 90
                            + 360
                        ) % 360
                        # If the drive input is not within 180 degrees of the angle to the note
                        if (direction_angle >= angle_to_nearest_algae + 90) or (
                            direction_angle <= angle_to_nearest_algae - 90
                        ):
                            # Drive with the driver input
                            self.robot.drivetrain.drive_with_pid(
                                Translation2d(forward_back, left_right)
                                * const.SWERVE_MAX_SPEED,
                                angle_to_nearest_algae,
                            )
                        else:  # Drive toward the algae
                            note_front_back = drive_magnitude * math.cos(
                                math.radians(angle_to_nearest_algae)
                            )
                            note_left_right = drive_magnitude * math.sin(
                                math.radians(angle_to_nearest_algae)
                            )
                            self.robot.drivetrain.drive_with_pid(
                                Translation2d(note_front_back, note_left_right)
                                * const.SWERVE_MAX_SPEED,
                                angle_to_nearest_algae,
                            )
                            self.robot_oriented_angle = angle_to_nearest_algae
                    else:  # If the limelight does not see the note, drive normally
                        self.robot.leds.set_mode(robot.leds.MODE_INTAKING)
                        self.robot.drivetrain.drive_with_pid(
                            Translation2d(forward_back, left_right)
                            * const.SWERVE_MAX_SPEED,
                            self.robot_oriented_angle,
                        )
                else:
                    self.looking_for_algae = False
                    self.robot.drivetrain.drive_with_pid(
                        Translation2d(forward_back, left_right)
                        * const.SWERVE_MAX_SPEED,
                        self.robot_oriented_angle,
                    )

        @self.driver1.X.whenPressed  # Turn 90 degrees left
        def _():
            self.cardinal_directing = True
            self.cardinal = 270
            self.robot_oriented_angle = 270

        @self.driver1.B.whenPressed  # Turn 90 degrees right
        def _():
            self.cardinal_directing = True
            self.cardinal = 90
            self.robot_oriented_angle = 90

        @self.driver1.A.whenReleased  # Turn 180 degrees aways
        def _():
            self.cardinal_directing = True
            self.cardinal = 0
            self.robot_oriented_angle = 0

        @self.driver1.Y.whenPressed  # Turn 180 degrees towards
        def _():
            self.cardinal_directing = True
            self.cardinal = 180
            self.robot_oriented_angle = 180

        @self.driver1.POV.DOWN.whenPressed  # Reset Gyro
        def _():
            robot.poseEstimator.set_yaw(0.0)

        @self.driver1.POV.LEFT.whenHeld  # Code Crash Input 1
        def _():
            self.can_crash = True

        @self.driver1.POV.LEFT.whenReleased  # Code Crash Input 1
        def _():
            self.can_crash = False

        @self.driver1.START.whenPressed  # Code Crash Input 2
        def _():
            if self.can_crash:
                4096 / 0

        @self.driver2.A.whenPressed # Intaking algae
        def _():
            self.robot.leds.set_mode(robot.leds.MODE_INTAKING)
            self.looking_for_algae = True

    def log(self):
        pass
