# This is to help vscode
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot

import math
# from limelight import Limelight

from wpimath.trajectory import TrapezoidProfile
from wpilibextra.coroutine.coroutine_command import autoroutine2command
from wpilib import Timer
import wpimath.geometry
import const

# from commands import autonomous
# from commands.autonomous import DriveTrajectory
from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
)

# Example for when we begin working on autonomous mode
class AutoRoutines:
    """
    A class to hold all the autonomous routines.
    """

    def __init__(self, robot: "Robot"):
        self.robot = robot

        self.p_1 = self.robot.followPathCommand("3P_1")
        self.p_2 = self.robot.followPathCommand("3P_2")
        self.p_3 = self.robot.followPathCommand("3P_3")
        self.p_4 = self.robot.followPathCommand("3P_4")
        self.p_5 = self.robot.followPathCommand("3P_5")

    def three_piece_no_pathplanner(self):
        return SequentialCommandGroup(
            # self.robot.followPathCommand("3P_1"),
            self.robot.coroutines.score_left_branch_1,
            self.robot.coroutines.score_L4_1,
            self.robot.coroutines.score_3_piece_auto_no_closest_tag_1.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_1,
            # self.robot.followPathCommand("3P_2"),
            self.robot.coroutines.intake_coral_1,
            self.robot.coroutines.reset_robot_after_intaking_1,
            # self.robot.followPathCommand("3P_3"),
            self.robot.coroutines.score_right_branch,
            self.robot.coroutines.score_L4_2,
            self.robot.coroutines.score_3_piece_auto_no_closest_tag_2.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_2,
            # self.robot.followPathCommand("3P_4"),
            self.robot.coroutines.intake_coral_2,
            self.robot.coroutines.reset_robot_after_intaking_2,
            # self.robot.followPathCommand("3P_5"),
            self.robot.coroutines.score_left_branch_2,
            self.robot.coroutines.score_L4_3,
            self.robot.coroutines.score_3_piece_auto_no_closest_tag_3.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_3,
        )

    def four_piece_auto_test(self):
        return SequentialCommandGroup(
            self.robot.followPathCommand("Starting line to face 5"),
            self.robot.followPathCommand("face 5 to right source"),
            self.robot.followPathCommand("right source to face 6"),
            self.robot.followPathCommand("face 6 to right source"),
            self.robot.followPathCommand("right source to face 1"),
            self.robot.followPathCommand("face 1 to right source"),
            self.robot.followPathCommand("right source to face 6"),
        )

    def three_piece_auto(self):
        return SequentialCommandGroup(
            self.p_1,
            self.robot.coroutines.score_left_branch_1,
            self.robot.coroutines.score_L4_1,
            self.robot.coroutines.score_piece_1.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.p_2,
            self.robot.coroutines.intake_coral_1,
            self.robot.coroutines.reset_robot_after_intaking_1,
            self.p_3,
            self.robot.coroutines.score_right_branch,
            self.robot.coroutines.score_L4_2,
            self.robot.coroutines.score_piece_2.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.p_4,
            self.robot.coroutines.intake_coral_2,
            self.robot.coroutines.reset_robot_after_intaking_2,
            self.p_5,
            self.robot.coroutines.score_left_branch_2,
            self.robot.coroutines.score_L4_3,
            self.robot.coroutines.score_piece_3.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_3,
        )

    def tush_push_auto(self):
        return SequentialCommandGroup(
            self.robot.coroutines.tush_push_towards_yaw,
            self.robot.coroutines.score_left_branch_1,
            self.robot.coroutines.score_L4_1,
            self.robot.coroutines.score_piece_1.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.robot.followPathCommand("3P_2"),
            self.robot.coroutines.intake_coral_1,
            self.robot.coroutines.reset_robot_after_intaking_1,
            self.robot.followPathCommand("3P_3"),
            self.robot.coroutines.score_right_branch,
            self.robot.coroutines.score_L4_2,
            self.robot.coroutines.score_piece_2.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.robot.followPathCommand("3P_4"),
            self.robot.coroutines.intake_coral_2,
            self.robot.coroutines.reset_robot_after_intaking_2,
            self.robot.followPathCommand("3P_5"),
            self.robot.coroutines.score_left_branch_2,
            self.robot.coroutines.score_L4_3,
            self.robot.coroutines.score_piece_3.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_3,
        )