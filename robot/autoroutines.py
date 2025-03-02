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

    def three_piece_auto(self):
        return SequentialCommandGroup(
            self.robot.followPathCommand("3P_1"),
            self.robot.coroutines.score_L4,
            self.robot.corountines.score_left_branch,
            self.robot.coroutines.score_piece,
            self.robot.coroutines.reset_robot_after_scoring,
            self.robot.followPathCommand("3P_2"),
            self.robot.corountines.intake_coral,
            self.robot.followPathCommand("3P_3"),
            self.robot.coroutines.score_L4,
            self.robot.corountines.score_right_branch,
            self.robot.coroutines.score_piece,
            self.robot.coroutines.reset_robot_after_scoring,
            self.robot.followPathCommand("3P_4"),
            self.robot.corountines.intake_coral,
            self.robot.followPathCommand("3P_5"),
            self.robot.coroutines.score_L4,
            self.robot.corountines.score_left_branch,
            self.robot.coroutines.score_piece,
            self.robot.coroutines.reset_robot_after_scoring,
        )