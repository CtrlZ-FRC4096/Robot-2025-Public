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
        def reset_robot_after_scoring_1():
            yield
            robot.mechanisms_at_default = True
            robot.oi.running_pid_lineup = False
            robot.oi.score_intent = False
            robot.at_scoring_position = False
            robot.oi.robot_oriented_angle = robot.poseEstimator.getYaw().degrees()
            robot.end_effector.stop()
            robot.drivetrain.stop()
            robot.has_coral = False
            
        @commandify
        def reset_robot_after_scoring_2():
            yield
            robot.mechanisms_at_default = True
            robot.oi.running_pid_lineup = False
            robot.oi.score_intent = False
            robot.at_scoring_position = False
            robot.oi.robot_oriented_angle = robot.poseEstimator.getYaw().degrees()
            robot.end_effector.stop()
            robot.drivetrain.stop()
            robot.has_coral = False
            
        @commandify
        def reset_robot_after_scoring_3():
            yield
            robot.mechanisms_at_default = True
            robot.oi.running_pid_lineup = False
            robot.oi.score_intent = False
            robot.at_scoring_position = False
            robot.oi.robot_oriented_angle = robot.poseEstimator.getYaw().degrees()
            robot.end_effector.stop()
            robot.drivetrain.stop()
            robot.has_coral = False


        self.reset_robot_after_scoring_1 = (reset_robot_after_scoring_1)
        self.reset_robot_after_scoring_2 = (reset_robot_after_scoring_2)
        self.reset_robot_after_scoring_3 = (reset_robot_after_scoring_3)

        @commandify
        def score_left_branch_1():
            yield
            robot.oi.right_branch = False
            
        @commandify
        def score_left_branch_2():
            yield
            robot.oi.right_branch = False

        self.score_left_branch_1 = (
            score_left_branch_1
        )
        
        self.score_left_branch_2 = (
            score_left_branch_2
        )

        @commandify
        def score_right_branch():
            yield
            robot.oi.right_branch = True

        self.score_right_branch = (
            score_right_branch
        )

        @commandify
        def score_piece_1():
            yield
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.at_scoring_position = False
            robot.score_piece = False
            final_lineup_pose = robot.poseEstimator.get_path_to_reef(
                robot.poseEstimator.calculate_closest_reef_tag()[1],
            	robot.oi.right_branch,
                do_manip_offset=True
            )
            robot.mechanisms_at_default = False
            robot.oi.running_pid_lineup = True
            robot.oi.intent_to_auto_drive = True
            robot.oi.score_intent = True
            while True:
                yield
                robot.drivetrain.go_to_pose_profiled_pid(final_lineup_pose)
            
        @commandify
        def score_piece_2():
            yield
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.at_scoring_position = False
            robot.score_piece = False
            final_lineup_pose = robot.poseEstimator.get_path_to_reef(
                robot.poseEstimator.calculate_closest_reef_tag()[1],
            	robot.oi.right_branch,
                do_manip_offset=True
            )
            robot.mechanisms_at_default = False
            robot.oi.running_pid_lineup = True
            robot.oi.intent_to_auto_drive = True
            robot.oi.score_intent = True
            while True:
                yield
                robot.drivetrain.go_to_pose_profiled_pid(final_lineup_pose)
            
        @commandify
        def score_piece_3():
            yield
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.at_scoring_position = False
            robot.score_piece = False
            final_lineup_pose = robot.poseEstimator.get_path_to_reef(
                robot.poseEstimator.calculate_closest_reef_tag()[1],
            	robot.oi.right_branch,
                do_manip_offset=True
            )
            robot.mechanisms_at_default = False
            robot.oi.running_pid_lineup = True
            robot.oi.intent_to_auto_drive = True
            robot.oi.score_intent = True
            while True:
                yield
                robot.drivetrain.go_to_pose_profiled_pid(final_lineup_pose)

        self.score_piece_1 = (score_piece_1)
        self.score_piece_2 = (score_piece_2)
        self.score_piece_3 = (score_piece_3)

        @commandify
        def score_L4_1():
            yield
            robot.mechanisms_at_default = False
            robot.score_state = RobotScoringPositions.L4_Scoring
            robot.oi.score_intent = True
            
        @commandify
        def score_L4_2():
            yield
            robot.mechanisms_at_default = False
            robot.score_state = RobotScoringPositions.L4_Scoring
            robot.oi.score_intent = True
            
        @commandify
        def score_L4_3():
            yield
            robot.mechanisms_at_default = False
            robot.score_state = RobotScoringPositions.L4_Scoring
            robot.oi.score_intent = True

        self.score_L4_1 = (score_L4_1)
        self.score_L4_2 = (score_L4_2)
        self.score_L4_3 = (score_L4_3)

        @commandify
        def score_L3():
            yield
            robot.mechanisms_at_default = False
            robot.score_state = RobotScoringPositions.L3_Scoring
            robot.oi.score_intent = True

        self.score_L3 = (
            score_L3
        )

        @commandify
        def score_L2():
            yield
            robot.mechanisms_at_default = False
            robot.score_state = RobotScoringPositions.L2_Scoring
            robot.oi.score_intent = True

        self.score_L2 = (
            score_L2
        )

        @commandify
        def score_L1():
            yield
            robot.mechanisms_at_default = False
            robot.score_state = RobotScoringPositions.L1_Scoring
            robot.oi.score_intent = True

        self.score_L1 = (
            score_L1
        )

        @commandify # position 1 on source
        def set_position_to_1_on_source():
            yield
            robot.oi.position_on_source = 1

        self.set_position_to_1_on_source = (
            set_position_to_1_on_source
        )

        @commandify # position 2 on source
        def set_position_to_2_on_source():
            yield
            robot.oi.position_on_source = 2

        self.set_position_to_2_on_source = (
            set_position_to_2_on_source
        )

        @commandify # position 3 on source
        def set_position_to_3_on_source():
            yield
            robot.oi.position_on_source = 3

        self.set_position_to_3_on_source = (
            set_position_to_3_on_source
        )

        @commandify
        def intake_coral_1():
            yield
            final_lineup_pose = robot.poseEstimator.get_path_to_source(False, robot.oi.position_on_source)
            robot.mechanisms_at_default = False
            robot.at_scoring_position = False
            robot.score_piece = False
            robot.funnel_intake.is_intaking = True
            robot.end_effector.is_intaking = True
            robot.oi.running_pid_lineup = True
            robot.oi.intent_to_auto_drive = True
            robot.oi.score_intent = False
            while True:
                yield
                robot.drivetrain.go_to_pose_profiled_pid(final_lineup_pose)
            
        @commandify
        def intake_coral_2():
            yield
            final_lineup_pose = robot.poseEstimator.get_path_to_source(False, robot.oi.position_on_source)
            robot.mechanisms_at_default = False
            robot.at_scoring_position = False
            robot.score_piece = False
            robot.funnel_intake.is_intaking = True
            robot.end_effector.is_intaking = True
            robot.oi.running_pid_lineup = True
            robot.oi.intent_to_auto_drive = True

            robot.oi.score_intent = False
            while True:
                yield
                robot.drivetrain.go_to_pose_profiled_pid(final_lineup_pose)

        self.intake_coral_1 = (intake_coral_1)
        self.intake_coral_2 = (intake_coral_2)

        @commandify
        def reset_robot_after_intaking_1():
            yield
            robot.mechanisms_at_default = True
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.oi.running_pid_lineup = False
            robot.oi.intent_to_auto_drive = False
            robot.oi.score_intent = False
            robot.oi.robot_oriented_angle = robot.poseEstimator.getYaw().degrees()
            robot.drivetrain.stop()
            robot.has_coral = True
            
        @commandify
        def reset_robot_after_intaking_2():
            yield
            robot.mechanisms_at_default = True
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.oi.running_pid_lineup = False
            robot.oi.intent_to_auto_drive = False
            robot.oi.score_intent = False
            robot.oi.robot_oriented_angle = robot.poseEstimator.getYaw().degrees()
            robot.drivetrain.stop()
            robot.has_coral = True

        self.reset_robot_after_intaking_1 = (reset_robot_after_intaking_1)
        self.reset_robot_after_intaking_2 = (reset_robot_after_intaking_2)
        
        

