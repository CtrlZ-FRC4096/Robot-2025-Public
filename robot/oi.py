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

        self.rumble_button = Button(lambda: self.robot.has_note)
        self.can_crash = False

        self.find_heading = True
        self.tick_count = 0

        self.face = 1
        self.right_branch = True
        self.tag_to_pathfind = 0
        self.pathfind_to_reef = None
        self.running_path = False
        self.running_pid = False
        self.target_pose = Pose2d()
        self.path_to_reef = []
        # self.astar_count = 0
        # self.astar_next_point = False
        self.run_path = True

        self.pathfinding_constraints = PathConstraints(
            4.0, 4.0, 3.0 * math.pi, 3.0 * math.pi
        )

        self.target_pose = self.robot.poseEstimator.get_path_to_reef(
                3,
                False,
                24,
                False
            )
        print("before path")
        path = PathGenerator(self.robot.drivetrain.curPose, self.target_pose, False)
        print("before smooth")
        # smooth_path = [point for point in reversed(path.getSmoothPath())]

        # for idx in range(len(smooth_path)):
        #     if idx == 0:
        #         self.path_to_reef.append(smooth_path[-1])
        #     elif idx == len(smooth_path) - 1:
        #         self.path_to_reef.append(smooth_path[0])
        #     else:
        #         self.path_to_reef.append(smooth_path[idx])
        self.path_to_reef = path.getSmoothPath()
        print(self.path_to_reef)
        print("before pursuit")
        self.pure_pursuit = PurePursuitController(0.05, self.path_to_reef)
        print("after pursuit")

        tgt_pose = self.robot.poseEstimator.field.getObject("tgt pose")
        tgt_pose.setPose(self.target_pose)

        for idx in range(len(self.path_to_reef)):
            # if path.inObstacle(self.path_to_reef[idx]):
                field_object = self.robot.poseEstimator.field_for_single_tag.getObject("point " + str(idx))
                field_object.setPose(Pose2d(self.path_to_reef[idx], Rotation2d.fromDegrees(0)))

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

                if (self.running_path) and (
                    abs(self.driver1.LEFT_JOY_X()) > 0.05
                    or abs(self.driver1.LEFT_JOY_Y()) > 0.05
                    or abs(self.driver1.RIGHT_JOY_X()) > 0.1
                    or abs(self.driver1.RIGHT_JOY_Y()) > 0.1
                ):
                    self.robot.scheduler.cancelAll()
                    self.running_path = False

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
                elif self.run_path: #or self.running_pid:
                    # if self.astar_next_point:
                    #     self.astar_next_point = False
                    #     self.astar_count += 1
                    # if self.astar_count == len(self.path_to_reef) - 1:
                    #self.pure_pursuit.last_lookahead_point = self.robot.drivetrain.curPose.translation()
                    print("driving with profiled, ", self.pure_pursuit.getLookaheadIntersectionAllPath(self.robot.drivetrain.curPose))
                    if self.pure_pursuit.getVelocities(self.robot.drivetrain.curPose) == False:
                        self.run_path = False
                    else:
                        vx = self.pure_pursuit.getVelocities(self.robot.drivetrain.curPose)[0]
                        vy = self.pure_pursuit.getVelocities(self.robot.drivetrain.curPose)[1]
                        self.robot.drivetrain.drive(Translation2d(vx, vy), 0, True, False)
                        # self.run_path = False
                    # else:
                    #     self.robot.drivetrain.go_to_pose_profiled_pid(self.path_to_reef[self.astar_count], False)
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

        @self.driver1.Y.whenReleased  # Turn 180 degrees aways
        def _():
            self.cardinal_directing = True
            self.cardinal = 0
            self.robot_oriented_angle = 0

        @self.driver1.A.whenPressed  # Turn 180 degrees towards
        def _():
            self.cardinal_directing = True
            self.cardinal = 180
            self.robot_oriented_angle = 180

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

        @self.driver2.POV.UP.whenPressed  # Run funnel intake
        def _():
            self.robot.funnel_intake.is_running = True


        # @self.driver1.LEFT_TRIGGER_AS_BUTTON.whenReleased  # stop pathfinnd
        # def _():
        #     self.robot.scheduler.cancelAll()
        #     self.running_path = False
        #     self.robot.poseEstimator.score_intent = False

        @self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenHeld  # Run profiled PID to tag
        def _():
            self.target_pose = self.robot.poseEstimator.get_path_to_reef(
                self.robot.poseEstimator.calculate_closest_reef_tag()[1],
                self.right_branch,
            )
            path = PathGenerator(self.robot.poseEstimator.curEstPose, self.target_pose)
            self.path_to_reef = path.getPointList()
            self.running_pid = True
            self.robot.poseEstimator.score_intent = True

        @self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenReleased  # stop profiled PID
        def _():
            self.running_pid = False
            self.robot.poseEstimator.score_intent = False
            self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()
            # self.robot.drivetrain.stop() May or may not be needed to stop the robot from tracking the PID

        @self.driver2.RIGHT_TRIGGER_AS_BUTTON.whenPressed  # right face
        def _():
            self.right_branch = True

        @self.driver2.LEFT_TRIGGER_AS_BUTTON.whenPressed  # left face
        def _():
            self.right_branch = False

        # @self.driver2.A.whenPressed # face 6
        # def _():
        #     self.face = 6

        @self.driver2.POV.RIGHT.whenPressed  # STOP ALL SUBSYSTEMS
        def _():
            self.robot.stop_all_subsystems()

    def log(self):
        SmartDashboard.putNumber("robot oriented angle", self.robot_oriented_angle)
