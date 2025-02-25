# This is to help vscode
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot
# ------------------------------------------------

from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
)

import math
from wpilib import Timer

from wpilibextra.coroutine import commandify
import oi
from robot_scoring_positions import RobotScoringPositions


class Coroutines:
    """
    Coroutines - This class defines coroutines commands that are used in robot code.
    """

    def __init__(self, robot: "Robot"):
        @commandify
        def reset_robot_after_scoring():
            yield
            yield from robot.wait(0.25) # wait so that robot can fully outtake piece (we can adjust this time later)
            robot.mechanisms_at_default = True
            robot.oi.score_intent = False
        
        self.reset_robot_after_scoring = (
            reset_robot_after_scoring()
        )

        @commandify
        def score_piece():
            yield
            robot.mechanisms_at_default = False
            robot.score_piece = True

        self.score_piece = (
            score_piece()
        )

        @commandify
        def score_L4():
            yield
            robot.mechanisms_at_default = False
            robot.score_state = RobotScoringPositions.L4_Scoring
            robot.oi.score_intent = True
        
        self.score_L4 = (
            score_L4()
        )

        @commandify
        def score_L3():
            yield
            robot.mechanisms_at_default = False
            robot.score_state = RobotScoringPositions.L3_Scoring
            robot.oi.score_intent = True

        self.score_L3 = (
            score_L3()
        )
        
        @commandify
        def score_L2():
            yield
            robot.mechanisms_at_default = False
            robot.score_state = RobotScoringPositions.L2_Scoring
            robot.oi.score_intent = True
        
        self.score_L2 = (
            score_L2()
        )
        
        @commandify
        def score_L1():
            yield
            robot.mechanisms_at_default = False
            robot.score_state = RobotScoringPositions.L1_Scoring
            robot.oi.score_intent = True

        self.score_L1 = (
            score_L1()
        )

        @commandify
        def intake_coral():
            yield
            robot.mechanisms_at_default = False
            robot.funnel_intake.is_intaking = True
            robot.end_effector.is_intaking = True
            yield from robot.wait(1.0) # wait for coral to be intaken
        
        self.intake_coral = (
            intake_coral()
        )