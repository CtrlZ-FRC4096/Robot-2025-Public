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

from wpilib import SmartDashboard
from robot_scoring_positions import RobotScoringPositions
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


from pathplannerlib.auto import (
    AutoBuilder,
    PathPlannerAuto,
    NamedCommands,
    PathConstraints,
    PathPlannerPath,
)
from pathplannerlib.path import (
    GoalEndState,
    Waypoint,
    IdealStartingState,
)

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


# Controls
from wpilibextra.customcontroller import XboxCommandController

from field_const import FieldConstants
from path_gen import PathGenerator, PurePursuitController
from wpimath.units import inchesToMeters, degreesToRadians

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

        self.rumble_button = Button(lambda: self.robot.has_coral)
        self.can_crash = False

        self.find_heading = True
        self.tick_count = 0

        self.face = 1
        self.right_branch = True
        self.running_pid_lineup = False
        self.final_lineup_pose = Pose2d()
        self.score_intent = False
        self.position_on_source = 2

        self.manual_scoring = False

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

                # Cancel drive with pid if robot is moving manually
                # if (self.running_pid_lineup) and (
                #     abs(self.driver1.LEFT_JOY_X()) > 0.05
                #     or abs(self.driver1.LEFT_JOY_Y()) > 0.05
                #     or abs(self.driver1.RIGHT_JOY_X()) > 0.1
                #     or abs(self.driver1.RIGHT_JOY_Y()) > 0.1
                # ):
                #     self.running_pid_lineup = False

                if abs(rotate) >= 0.02:
                    self.cardinal_directing = False
                    self.find_heading = True
                    self.wait_one_tick = False
                    self.tick_count = 0
                    self.robot.drivetrain.drive(
                        Translation2d(forward_back, left_right)
                        * const.SWERVE_MAX_SPEED,
                        rotate * 3.0,
                        True,
                        False,
                    )
                    self.robot_oriented_angle = (
                        self.robot.poseEstimator.getYaw().degrees()
                    )
                elif self.running_pid_lineup:
                    self.robot.drivetrain.go_to_pose_profiled_pid(self.final_lineup_pose)
                else:
                    # if not self.cardinal_directing:
                    #     if self.find_heading:
                    #         if self.wait_one_tick:
                    #             self.robot_oriented_angle = (
                    #                 self.robot.poseEstimator.getYaw().degrees()
                    #             )
                    #             self.find_heading = False
                    #         else:
                    #             self.wait_one_tick = True
                    if not self.cardinal_directing:
                        if self.find_heading:
                            if self.tick_count <= 5:
                                self.robot_oriented_angle = (
                                    self.robot.poseEstimator.getYaw().degrees()
                                )
                                self.tick_count += 1
                            else:
                                self.find_heading = False
                    self.robot.drivetrain.drive_with_pid(
                        Translation2d(forward_back, left_right)
                        * const.SWERVE_MAX_SPEED,
                        self.robot_oriented_angle,
                    )

        @self.driver1.X.whenPressed  # Manual elevator raise
        def _():
            self.manual_scoring = True
            self.robot.mechanisms_at_default = False

        @self.driver1.B.whenHeld # outtake piece
        def _():
            self.robot.mechanisms_at_default = False
            self.robot.score_piece = True

        @self.driver1.B.whenReleased
        def _():
            self.robot.score_piece = False
            self.robot.mechanisms_at_default = True
            self.manual_scoring = False

        @self.driver1.Y.whenPressed
        def _():
            self.robot.mechanisms_at_default = True
            self.manual_scoring = False

        @self.driver1.POV.DOWN.whenPressed  # Reset Gyro
        def _():
            robot.poseEstimator.set_yaw(0.0)
            self.robot_oriented_angle = 0.0

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

        @self.driver1.RIGHT_BUMPER.whenPressed  # begin intake process
        def _():
            self.robot.mechanisms_at_default = False
            self.robot.funnel_intake.is_intaking = True
            self.robot.end_effector.is_intaking = True
            self.robot.at_scoring_position = False
            self.score_intent = False
            self.robot.score_piece = False

        @self.driver1.LEFT_BUMPER.whenPressed  # manual end to intake process
        def _():
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False
            self.robot.mechanisms_at_default = True

        @self.driver1.LEFT_TRIGGER_AS_BUTTON.whenHeld #run profiled pid to nearest source
        def _():
            self.robot.mechanisms_at_default = False
            self.robot.at_scoring_position = False
            self.robot.score_piece = False
            self.robot.funnel_intake.is_intaking = True
            self.robot.end_effector.is_intaking = True
            self.running_pid_lineup = True
            self.score_intent = False
            self.final_lineup_pose = self.robot.poseEstimator.get_path_to_source(False, self.position_on_source)

        @self.driver1.LEFT_TRIGGER_AS_BUTTON.whenReleased #stop pid
        def _():
            self.robot.mechanisms_at_default = True
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False
            self.running_pid_lineup = False
            self.score_intent = False
            self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()
            self.robot.drivetrain.stop()

        @self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenHeld  # Run profiled PID to tag
        def _():
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False
            self.robot.at_scoring_position = False
            self.robot.score_piece = False
            self.final_lineup_pose = self.robot.poseEstimator.get_path_to_reef(
                self.robot.poseEstimator.calculate_closest_reef_tag()[1],
                self.right_branch,
                do_manip_offset=True
            )
            self.robot.mechanisms_at_default = False
            self.running_pid_lineup = True
            self.score_intent = True

        @self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenReleased  # stop profiled PID
        def _():
            self.robot.mechanisms_at_default = True
            self.running_pid_lineup = False
            self.score_intent = False
            self.robot.at_scoring_position = False
            self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()
            self.robot.end_effector.stop()
            self.robot.drivetrain.stop() #May or may not be needed to stop the robot from tracking the PID

        @self.driver2.Y.whenPressed # L4
        def _():
            self.robot.score_state = RobotScoringPositions.L4_Scoring

        @self.driver2.B.whenPressed #L3
        def _():
            self.robot.score_state = RobotScoringPositions.L3_Scoring

        @self.driver2.A.whenPressed # L2
        def _():
            self.robot.score_state = RobotScoringPositions.L2_Scoring

        @self.driver2.X.whenPressed # L1
        def _():
            self.robot.score_state = RobotScoringPositions.L1_Scoring

        @self.driver2.RIGHT_TRIGGER_AS_BUTTON.whenPressed  # right face
        def _():
            self.right_branch = True

        @self.driver2.LEFT_TRIGGER_AS_BUTTON.whenPressed  # left face
        def _():
            self.right_branch = False

        @self.driver2.POV.LEFT.whenPressed # position 1 on source
        def _():
            self.position_on_source = 1

        @self.driver2.POV.UP.whenPressed # position 2 on source
        def _():
            self.position_on_source = 2

        @self.driver2.POV.RIGHT.whenPressed # position 3 on source
        def _():
            self.position_on_source = 3

        @self.driver1.BACK.whenPressed
        def _():
            self.robot.mechanisms_at_default = True
            self.score_intent = False
            self.manual_scoring = False
            self.running_pid_lineup = False
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False


        # @self.driver2.POV.DOWN.whenPressed
        # def _():
        #     self.robot.mechanisms_at_default = False
        #     self.robot.elevator.set_elevator_height(56.0)


    def log(self):
        SmartDashboard.putNumber("robot oriented angle", self.robot_oriented_angle)
        SmartDashboard.putNumber("position on source", self.position_on_source)
        SmartDashboard.putBoolean("right branch", self.right_branch)
        SmartDashboard.putNumber("final lineup x", self.final_lineup_pose.X())
        SmartDashboard.putNumber("final lineup y", self.final_lineup_pose.Y())