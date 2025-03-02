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
            self.robot.followPathCommand("3P_1"),
            lambda: self.robot.coroutines.score_L4,  
            lambda: self.robot.coroutines.score_left_branch,
            lambda: self.robot.coroutines.score_piece,
            lambda: self.robot.coroutines.reset_robot_after_scoring,
            self.robot.followPathCommand("3P_2"),
            lambda: self.robot.coroutines.intake_coral.until(lambda: self.robot.has_coral).withTimeout(3.0),
            self.robot.followPathCommand("3P_3"),
            lambda: self.robot.coroutines.score_L4,
            lambda: self.robot.coroutines.score_right_branch,
            lambda: self.robot.coroutines.score_piece,
            lambda: self.robot.coroutines.reset_robot_after_scoring,
            self.robot.followPathCommand("3P_4"),
            lambda: self.robot.coroutines.intake_coral.until(lambda: self.robot.has_coral).withTimeout(3.0),
            self.robot.followPathCommand("3P_5"),
            lambda: self.robot.coroutines.score_L4,
            lambda: self.robot.coroutines.score_left_branch,
            lambda: self.robot.coroutines.score_piece,
            lambda: self.robot.coroutines.reset_robot_after_scoring,
        )