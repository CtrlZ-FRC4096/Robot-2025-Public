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
from wpimath.units import degreesToRadians
import const
from pathplannerlib.path import PathConstraints, PathPlannerPath

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
        #path_constraints = PathConstraints()# adjust these max speeds and accelerations for each path
        #self.p_1 = self.robot.followPathCommand("3P_1")
        self.p_2 = self.robot.followPathCommand("3P_2")
        self.p_3 = self.robot.followPathCommand("3P_3")
        self.p_4 = self.robot.followPathCommand("3P_4")
        self.p_5 = self.robot.followPathCommand("3P_5")
        self.p_6 = self.robot.followPathCommand("3P_6")
        self.p_7 = self.robot.followPathCommand("3P_7")
        self.p_2_f5 = self.robot.followPathCommand("f5_intake")
        
        self.p_1_2p = self.robot.followPathCommand("2P_1",  PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540)))
        self.p_2_2p = self.robot.followPathCommand("2P_2",  PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540)))
        self.p_3_2p = self.robot.followPathCommand("2P_3",  PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540)))
        self.p_3p_f1_3 = self.robot.followPathCommand("3P_F1_3")
        self.p_3p_f1_4 = self.robot.followPathCommand("3P_F1_4")
        self.p_3p_f1_5 = self.robot.followPathCommand("3P_F1_5")
        self.p_for_2p = [
            self.p_1_2p,
            self.p_2_2p,
            self.p_3_2p,
        ]
        self.p_for_3p_f1 = [
            self.p_2_f5,
            self.p_6,
            self.p_3p_f1_3,
            self.p_3p_f1_4,
            self.p_3p_f1_5,
        ]
        self.p_for_f5 = [
            self.p_2_f5,
            self.p_3,
            self.p_5,
            self.p_7,
        ]
        self.p_for_3p = [
            self.p_2,
            self.p_3,
            self.p_5,
            self.p_6,
        ]


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

    def three_piece_f1(self):
        return SequentialCommandGroup(
            self.robot.coroutines.score_piece_1.withTimeout(1.5),
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.p_for_3p_f1[0],
            self.robot.coroutines.intake_coral_1,
            self.p_for_3p_f1[1],
            self.robot.coroutines.score_piece_2,
            self.robot.coroutines.reset_robot_after_scoring_2,
            # self.p_for_3p_f1[2],
            self.robot.coroutines.intake_coral_2,
            self.p_for_3p_f1[3],
            self.robot.coroutines.score_piece_3,
            self.robot.coroutines.reset_robot_after_scoring_3,
            # self.p_for_3p_f1[4],
            self.robot.coroutines.intake_coral_3,
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
    def two_piece_delayed(self):
        return SequentialCommandGroup(
            self.robot.coroutines.score_piece_1.withTimeout(1.4),
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.robot.coroutines.wait_2p_after_score_1,
            self.p_for_2p[0],
            self.robot.coroutines.intake_coral_2p_1,
            self.p_for_2p[1],
            self.robot.coroutines.score_piece_2,
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.p_for_2p[2],
        )

    def three_piece_auto(self):
        return SequentialCommandGroup(
            self.robot.coroutines.score_piece_1.withTimeout(1.37),
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.p_for_3p[0],
            self.robot.coroutines.intake_coral_1,
            self.p_for_3p[1],
            self.robot.coroutines.score_piece_2,
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.robot.coroutines.intake_coral_2,
            self.p_for_3p[2],
            self.robot.coroutines.score_piece_3,
            self.robot.coroutines.reset_robot_after_scoring_3,
            self.robot.coroutines.intake_coral_3,
            self.p_for_3p[3],
            self.robot.coroutines.score_piece_4,
        )

    def three_piece_to_f5(self):
        return SequentialCommandGroup(
            self.robot.coroutines.score_piece_1_f5.withTimeout(1.9), #CHANGE TIME
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.p_for_f5[0],
            self.robot.coroutines.intake_coral_1,
            self.p_for_f5[1],
            self.robot.coroutines.score_piece_2,
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.robot.coroutines.intake_coral_2,
            self.p_for_f5[2],
            self.robot.coroutines.score_piece_3,
            self.robot.coroutines.reset_robot_after_scoring_3,
            # self.p_for_f5[3],
            self.robot.coroutines.intake_coral_3

        )

    def tush_push_auto(self):
        return SequentialCommandGroup(
            self.robot.coroutines.tush_push_towards_yaw,
            self.robot.coroutines.score_piece_1.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.robot.followPathCommand("3P_2"),
            self.robot.coroutines.intake_coral_1,
            self.robot.coroutines.reset_robot_after_intaking_1,
            self.robot.followPathCommand("3P_3"),
            self.robot.coroutines.score_piece_2.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.robot.followPathCommand("3P_4"),
            self.robot.coroutines.intake_coral_2,
            self.robot.coroutines.reset_robot_after_intaking_2,
            self.robot.followPathCommand("3P_5"),
            self.robot.coroutines.score_piece_3.withTimeout(5.0),
            self.robot.coroutines.reset_robot_after_scoring_3,
        )