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
                    forward_back *= 1.0
                    left_right *= 1.0

                rotate = -self.driver1.RIGHT_JOY_X()

                elevator_height_adjustment = -square(self.driver2.RIGHT_JOY_Y()) * const.ELEVATOR_RAISE_SPEED
                # if (self.robot.manual_scoring or self.robot.score_intent) and (abs(elevator_height_adjustment) > 0.05):
                #     if self.robot.end_effector.get_position() >= RobotScoringPositions.min_end_effector_position_to_move_elevator_up:
                #         self.robot.elevator.set_elevator_height(self.robot.elevator.get_height() + elevator_height_adjustment) # being overriden in elevator periodic
                #     else:
                #         self.robot.end_effector.set_end_effector_position(self.robot.score_state.end_effector_position)

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
                elif self.robot.running_pid_lineup:
                    # Cancel drive with pid if robot is moving manually
                    if (
						abs(self.driver1.LEFT_JOY_X()) > 0.05
						or abs(self.driver1.LEFT_JOY_Y()) > 0.05
						or abs(self.driver1.RIGHT_JOY_X()) > 0.1
						or abs(self.driver1.RIGHT_JOY_Y()) > 0.1
					):
                        forward_back *= 0.22
                        left_right *= 0.22
                        self.robot.drivetrain.go_to_pose_profiled_pid(self.robot.final_lineup_pose, forward_back, left_right, rotate)
                    else:
                        self.robot.drivetrain.go_to_pose_profiled_pid(self.robot.final_lineup_pose)
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
            self.robot.leds.mode = self.robot.leds.MODE_LOCKED_ON
            self.robot.raise_elevator_slightly_for_L1 = False
            self.robot.manual_scoring = True
            self.robot.mechanisms_at_default = False

        @self.driver1.Y.whenPressed
        def _():
            self.robot.mechanisms_at_default = True
            self.robot.raise_elevator_slightly_for_L1 = False
            self.robot.manual_scoring = False
            self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY

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
            self.robot.descoring_algae = False
            self.robot.raise_elevator_slightly_for_L1 = False
            self.robot.leds.mode = self.robot.leds.MODE_INTAKING
            self.robot.mechanisms_at_default = False
            self.robot.funnel_intake.is_intaking = True
            self.robot.end_effector.is_intaking = True
            self.robot.at_scoring_position = False
            self.robot.score_intent = False
            self.robot.score_piece = False

        @self.driver1.LEFT_BUMPER.whenPressed  # manual end to intake process
        def _():
            self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False
            self.robot.mechanisms_at_default = True

        @self.driver1.LEFT_TRIGGER_AS_BUTTON.whenHeld #run profiled pid to nearest source
        def _():
            self.robot.descoring_algae = False
            self.robot.raise_elevator_slightly_for_L1 = False
            self.robot.leds.mode = self.robot.leds.MODE_INTAKING
            self.robot.mechanisms_at_default = False
            self.robot.at_scoring_position = False
            self.robot.score_piece = False
            self.robot.funnel_intake.is_intaking = True
            self.robot.end_effector.is_intaking = True
            self.robot.running_pid_lineup = True
            self.robot.score_intent = False
            self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_source(self.robot.poseEstimator.calculate_closest_source()[0], self.robot.position_on_source)

        @self.driver1.LEFT_TRIGGER_AS_BUTTON.whenReleased #stop pid
        def _():
            self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
            self.robot.mechanisms_at_default = True
            self.robot.running_pid_lineup = False
            self.robot.score_intent = False
            self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()
            self.robot.drivetrain.stop()

        @self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenHeld  # Run profiled PID to tag
        def _():
            self.robot.descoring_algae = False
            self.robot.leds.mode = self.robot.leds.MODE_LOCKED_ON
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False
            self.robot.raise_elevator_slightly_for_L1 = False
            self.robot.at_scoring_position = False
            self.robot.score_piece = False
            if self.robot.score_state.number == 1:
                # self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_closest_L1()
                self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_reef(False, self.robot.poseEstimator.calculate_closest_reef_tag()[1], self.robot.right_branch, do_manip_offset=False, do_side_offset=True)
            else:
                self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_reef(
                    True, # change to true if wanting to use calibrated field
                    self.robot.poseEstimator.calculate_closest_reef_tag()[1],
                    self.robot.right_branch,
                    do_manip_offset=True,
                )
            self.robot.mechanisms_at_default = False
            self.robot.running_pid_lineup = True
            self.robot.score_intent = True

        @self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenReleased  # stop profiled PID
        def _():
            self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
            self.robot.raise_elevator_slightly_for_L1 = False
            self.robot.mechanisms_at_default = True
            self.robot.running_pid_lineup = False
            self.robot.score_intent = False
            self.robot.at_scoring_position = False
            self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()
            self.robot.end_effector.stop()
            self.robot.drivetrain.stop() # May or may not be needed to stop the robot from tracking the PID

        @self.driver2.Y.whenPressed # L4
        def _():
            if self.robot.is_climbing:
                self.robot.retract_climber = False
            else:
                self.robot.score_state = RobotScoringPositions.L4_Scoring

        @self.driver2.B.whenPressed #L3
        def _():
            self.robot.score_state = RobotScoringPositions.L3_Scoring

        @self.driver2.A.whenPressed # L2 and retract climber
        def _():
            if self.robot.is_climbing:
                self.robot.retract_climber = True
            else:
                self.robot.score_state = RobotScoringPositions.L2_Scoring

        @self.driver2.X.whenPressed # L1
        def _():
            self.robot.score_state = RobotScoringPositions.L1_Scoring

        @self.driver2.RIGHT_TRIGGER_AS_BUTTON.whenPressed  # right face
        def _():
            self.robot.right_branch = True

        @self.driver2.LEFT_TRIGGER_AS_BUTTON.whenPressed  # left face
        def _():
            self.robot.right_branch = False

        @self.driver2.POV.LEFT.whenPressed # position 1 on source
        def _():
            self.robot.position_on_source = 1

        @self.driver2.POV.UP.whenPressed # position 2 on source
        def _():
            self.robot.position_on_source = 2

        @self.driver2.POV.RIGHT.whenPressed # position 3 on source
        def _():
            self.robot.position_on_source = 3

        @self.driver2.POV.DOWN.whenHeld # outtake piece
        def _():
            self.robot.mechanisms_at_default = False
            self.robot.score_piece = True

        @self.driver2.POV.DOWN.whenReleased
        def _():
            self.robot.score_piece = False
            self.robot.mechanisms_at_default = True
            self.robot.manual_scoring = False

        @self.driver1.BACK.whenPressed
        def _():
            self.robot.mechanisms_at_default = True
            self.robot.score_intent = False
            self.robot.manual_scoring = False
            self.robot.running_pid_lineup = False
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False
            self.robot.descoring_algae = False

        @self.driver2.BACK.whenPressed # raise all setpoints
        def _():
            RobotScoringPositions.elevator_intake_height += 0.1
            RobotScoringPositions.elevator_climb_height += 0.2
            RobotScoringPositions.L1_Scoring.elevator_height += 0.5
            RobotScoringPositions.L2_Scoring.elevator_height += 0.5
            RobotScoringPositions.L3_Scoring.elevator_height += 0.5
            RobotScoringPositions.L4_Scoring.elevator_height += 0.5
            RobotScoringPositions.Descore_Algae_L3.elevator_height += 0.5
            RobotScoringPositions.Descore_Algae_L2.elevator_height += 0.5

        @self.driver2.START.whenPressed # lower all setpoints
        def _():
            RobotScoringPositions.elevator_intake_height -= 0.1
            RobotScoringPositions.elevator_climb_height -= 0.2
            RobotScoringPositions.L1_Scoring.elevator_height -= 0.5
            RobotScoringPositions.L2_Scoring.elevator_height -= 0.5
            RobotScoringPositions.L3_Scoring.elevator_height -= 0.5
            RobotScoringPositions.L4_Scoring.elevator_height -= 0.5
            RobotScoringPositions.Descore_Algae_L3.elevator_height -= 0.5
            RobotScoringPositions.Descore_Algae_L2.elevator_height -= 0.5

        @self.driver2.RIGHT_TRIGGER_AS_BUTTON.whenPressed # ignore
        def _():
            self.robot.end_effector_canrange_for_reef_returning_bad_values = not self.robot.end_effector_canrange_for_reef_returning_bad_values

        @self.driver2.LEFT_BUMPER.whenHeld # descore algae
        def _():
            self.robot.mechanisms_at_default = False
            self.robot.score_intent = False
            self.robot.manual_scoring = False
            self.robot.running_pid_lineup = False
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False
            self.robot.manual_scoring = True
            self.robot.score_piece = True
            self.robot.descoring_algae = True

            algae_height_at_closest_side = self.robot.poseEstimator.calculate_algae_height_at_closest_side()
            if algae_height_at_closest_side == 3:
                self.robot.score_state = RobotScoringPositions.Descore_Algae_L3
            elif algae_height_at_closest_side == 2:
                self.robot.score_state = RobotScoringPositions.Descore_Algae_L2

        @self.driver2.LEFT_BUMPER.whenReleased
        def _():
            self.robot.descoring_algae = False
            self.robot.mechanisms_at_default = True
            self.robot.score_intent = False
            self.robot.manual_scoring = False
            self.robot.running_pid_lineup = False
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False
            self.robot.manual_scoring = False
            self.robot.score_piece = False

            self.robot.score_state = RobotScoringPositions.L4_Scoring

        @self.driver2.RIGHT_BUMPER.whenHeld # climb
        def _():
            self.robot.mechanisms_at_default = False
            self.robot.score_intent = False
            self.robot.manual_scoring = False
            self.robot.running_pid_lineup = False
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False
            self.robot.is_climbing = True

        @self.driver2.RIGHT_BUMPER.whenReleased
        def _():
            self.robot.mechanisms_at_default = True
            self.robot.score_intent = False
            self.robot.manual_scoring = False
            self.robot.running_pid_lineup = False
            self.robot.funnel_intake.is_intaking = False
            self.robot.end_effector.is_intaking = False
            self.robot.is_climbing = False
            self.robot.retract_climber = False



    def log(self):
        SmartDashboard.putNumber("robot oriented angle", self.robot_oriented_angle)
        SmartDashboard.putNumber("position on source", self.robot.position_on_source)
        SmartDashboard.putBoolean("right branch", self.robot.right_branch)
        SmartDashboard.putNumber("final lineup x", self.robot.final_lineup_pose.X())
        SmartDashboard.putNumber("final lineup y", self.robot.final_lineup_pose.Y())