#! python3
"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2024
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

DEBUG = True

import logging

# Import our files


from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
    CommandScheduler,
)
import wpilib
from wpilib import Timer, DataLogManager, DriverStation, Field2d
import wpilib.sysid
import wpimath.geometry
import const
import oi
import math
import ntcore
import subsystems.climber
import subsystems.drivetrain

from wpimath.estimator import SwerveDrive4PoseEstimator

from phoenix6 import controls

# import subsystems.limelight
import subsystems.elevator
import subsystems.end_effector
import subsystems.funnel_intake
import subsystems.leds

import subsystems.limelight
import subsystems.poseEstimator

from wpilibextra.coroutine.coroutine_robot import CoroutineRobot
from wpilibextra.remote_shell import RemoteShell
from coroutines import Coroutines

import inspect
import autoroutines

from pathplannerlib.path import PathPlannerPath, Waypoint, IdealStartingState, GoalEndState, PathPoint
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands, FollowPathCommand, PathConstraints
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController

from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.units import degreesToRadians

from field_const import FieldConstants

from robot_scoring_positions import RobotScoringPositions
from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
)
from wpilibextra.coroutine import CoroutineCommand


log = logging.getLogger("robot")


class Robot(CoroutineRobot):
    """
    Main robot class.

    This is the central object, holding instances of all the robot subsystem
    and sensor classes.

    It also contains the methods for autonomous and
    teloperated modes, called during mode changes and repeatedly when those
    modes are active.

    The one instance of this class is also passed as an argument to the
    various other classes, so they have full access to all its properties.
    """

    def robot_start(self):
        # Networktables
        nt_inst = ntcore.NetworkTableInstance.getDefault()
        nt_inst.startServer()
        self.nt_robot = nt_inst.getTable("SmartDashboard")
        # self.nt_robot.putString('led_mode', 'off')

        # Match Stuff
        self.match_time = -1
        # const.IS_SIMULATION = self.isSimulation()

        self.has_coral = False

        # Command scheduler
        self.scheduler = CommandScheduler.getInstance()

        self.previously_scored = True
        
        # subsystems
        self.drivetrain = subsystems.drivetrain.Drivetrain(self)
        self.leds = subsystems.leds.LEDs(self)
        self.poseEstimator = subsystems.poseEstimator.PoseEstimator(self)
        self.funnel_intake = subsystems.funnel_intake.FunnelIntake(self)
        self.elevator = subsystems.elevator.Elevator(self)
        self.end_effector = subsystems.end_effector.EndEffector(self)
        # self.climber = subsystems.climber.Climber(self)

        self.subsystems = [
            self.drivetrain,
            self.leds,
            self.poseEstimator,
            self.funnel_intake,
            self.elevator,
            self.end_effector,
            # self.climber
        ]

        # If everything in self.subsystems is a Subsystem object, then
        # everything is automatically registered and this isn't needed.
        for subsystem in self.subsystems:
            self.scheduler.registerSubsystem(subsystem)

        # Coroutines
        self.coroutines = Coroutines(self)
        # With V's wrapper for commandify we automatically register all commands

        ### OTHER ###
        self.driverstation = wpilib.DriverStation
        self.oi = oi.OI(self)

		### STATE MACHINE ###
        self.score_state = RobotScoringPositions.L4_Scoring # defaulting to L4
        self.at_scoring_position = False

        self.score_piece = False
        self.mechanisms_at_default = True

        self.pathplanner_config = RobotConfig.fromGUISettings()

        self.match_time = -1

        ### PATHS ###
        self.p_2 = self.followPathCommand("3P_2")
        self.p_3 = self.followPathCommand("3P_3")
        self.p_4 = self.followPathCommand("3P_4")
        self.p_5 = self.followPathCommand("3P_5")
        self.p_6 = self.followPathCommand("3P_6")
        self.p_7 = self.followPathCommand("3P_7")
        self.p_2_f5 = self.followPathCommand("f5_intake")

        self.p_1_2p = self.followPathCommand("2P_1",  PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540)))
        self.p_2_2p = self.followPathCommand("2P_2",  PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540)))
        self.p_3_2p = self.followPathCommand("2P_3",  PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540)))
        self.p_3p_f1_3 = self.followPathCommand("3P_F1_3")
        self.p_3p_f1_4 = self.followPathCommand("3P_F1_4")
        self.p_3p_f1_5 = self.followPathCommand("3P_F1_5")
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

        ### FIELD LOGGING ###
        self.field = Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)
        ### LOGGING ###
        self.remote_shell = RemoteShell(self)

		# PATH CONSTRAINTS
        self.path_constraints = PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540))

        # self.flip_one_piece_f4(False)
        self.autoroutines = autoroutines.AutoRoutines(self)
        self.flip_3_piece_to_f5(False)
        self.auto = self.autoroutines.three_piece_to_f5()

        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

		### STATE MACHINE VARIABLES ###
        self.running_pid_lineup = False
        self.manual_scoring = False
        self.position_on_source = 1
        self.score_intent = False
        self.final_lineup_pose = Pose2d()
        self.right_branch = True
        self.previous_right_branch = True
        self.previous_position_on_source = 1

        self.end_effector_canrange_for_reef_returning_bad_values = False
        self.is_climbing = False
        self.retract_climber = False
        self.descoring_algae = False
        self.raise_elevator_slightly_for_L1 = False

        self.is_intaking = False
        self.raise_setpoints = 0.0


        @self.addPeriodic(period=0.25, offset=0)
        def _():
            self.log()
            pass

        @self.addPeriodic(period=0.05, offset=-0.01)
        def _leds():
            self.leds.periodicX()
            pass

        self.in_autonomous_mode = False

        while True:
            yield
            self.scheduler.run()


    def flip_X_coord(self, x):
        return FieldConstants.fieldLength - x
    def flip_Y_coord(self, y):
        return FieldConstants.fieldWidth - y

    def flip_path_cmd_across_x(self, cmd_path : FollowPathCommand):
        # FLIPPING Y COORDS
        path = cmd_path._originalPath
        waypoints = path.getWaypoints()
        new_waypoints = []
        ideal_start = IdealStartingState(path.getIdealStartingState().velocity, path.getIdealStartingState().rotation.__neg__())
        goal_end = GoalEndState(path.getGoalEndState().velocity, path.getGoalEndState().rotation.__neg__())
        for idx, waypoint in enumerate(waypoints):
            if idx == 0:
                # no prev control
                new_waypoints.append(Waypoint(
                    prevControl=None,
                    anchor=Translation2d(waypoint.anchor.X(), self.flip_Y_coord(waypoint.anchor.Y())),
                    nextControl=Translation2d(waypoint.nextControl.X(), self.flip_Y_coord(waypoint.nextControl.Y()))
                ))
            elif idx == len(waypoints) - 1:
                #no next control
                new_waypoints.append(Waypoint(
                    prevControl=Translation2d(waypoint.prevControl.X(), self.flip_Y_coord(waypoint.prevControl.Y())),
                    anchor=Translation2d(waypoint.anchor.X(), self.flip_Y_coord(waypoint.anchor.Y())),
                    nextControl=None
                ))
            else:
                        # both controls
                new_waypoints.append(Waypoint(
                    prevControl=Translation2d(waypoint.prevControl.X(), self.flip_Y_coord(waypoint.prevControl.Y())),
                        anchor=Translation2d(waypoint.anchor.X(), self.flip_Y_coord(waypoint.anchor.Y())),
                        nextControl=Translation2d(waypoint.nextControl.X(), self.flip_Y_coord(waypoint.nextControl.Y()))
                    ))
        new_path = PathPlannerPath(new_waypoints, self.path_constraints, ideal_starting_state=ideal_start, goal_end_state=goal_end)
        return FollowPathCommand(
            new_path,
            self.drivetrain.get_pose, # Robot pose supplier
            self.drivetrain.get_robot_relative_speeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.drivetrain.drive_robot_relative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
            PPHolonomicDriveController(  # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(
                    const.X_KP, const.X_KI, const.X_KD
                ),  # Translation PID constants
                PIDConstants(
                    const.THETA_KP, const.THETA_KI, const.THETA_KD
                ),  # Rotation PID constants
            ),
            self.pathplanner_config, # The robot configuration
            self.drivetrain.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self.drivetrain # Reference to this subsystem to set requirements
        )
    
    def flip_one_piece_f4(self, left_side : bool):
        if not left_side:
            self.score_1_face = 4
            self.score_1_right_branch = False
        else:
            self.score_1_face = 4
            self.score_1_right_branch = True
        self.poseEstimator.poseEst.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, FieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))
        self.poseEstimator.poseEstSingleTag.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, FieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))

    def flip_2_piece_delay_auto(self, left_side : bool):
        if not left_side:
            self.score_1_face = 4
            self.score_1_right_branch = False
            self.left_source_auto = False
            self.score_2_face = 4
            self.score_2_right_branch = True
            self.wait_score_1_2p = 0.0
            self.auto_position_source = 3
        else:
            self.left_source_auto = True
            self.score_1_face = 4
            self.score_1_right_branch = True
            self.score_2_face = 4
            self.score_2_right_branch = False
            self.wait_score_1_2p = 1.0
            self.auto_position_source = 3
            for idx, command in enumerate(self.p_for_2p):
                self.p_for_2p[idx] = self.flip_path_cmd_across_x(command)
        self.poseEstimator.poseEst.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, FieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))
        self.poseEstimator.poseEstSingleTag.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, FieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))


    def flip_3_piece_auto(self, left_side : bool):
        if not left_side:
            self.score_1_face = 4
            self.score_1_right_branch = False
            self.score_2_face = 6
            self.score_2_right_branch = True
            self.score_3_face = 6
            self.score_3_right_branch = False
            self.score_4_face = 1
            self.score_4_right_branch = True
            self.left_source_auto = False
            self.auto_position_source = 3
        else:
            self.left_source_auto = True
            self.score_1_face = 4
            self.score_1_right_branch = True
            self.score_2_face = 2
            self.score_2_right_branch = False
            self.score_3_face = 2
            self.score_3_right_branch = True
            self.score_4_face = 1
            self.score_4_right_branch = False
            self.auto_position_source = 3
            for idx, command in enumerate(self.p_for_3p):
                self.p_for_3p[idx] = self.flip_path_cmd_across_x(command)
        self.poseEstimator.poseEst.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, FieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))
        self.poseEstimator.poseEstSingleTag.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, FieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))

    def flip_3_piece_f1(self, left_side : bool):
        if not left_side:
            self.left_source_auto = False
            self.score_1_face = 5
            self.score_1_right_branch = False
            self.score_2_face = 1
            self.score_2_right_branch = True
            self.score_3_face = 1
            self.score_3_right_branch = False
            self.auto_position_source = 3
            self.poseEstimator.poseEst.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, 1.372, Rotation2d.fromDegrees(270))))
            self.poseEstimator.poseEstSingleTag.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, 1.372, Rotation2d.fromDegrees(270))))
        else:
            self.left_source_auto = True
            self.score_1_face = 3
            self.score_1_right_branch = True
            self.score_2_face = 1
            self.score_2_right_branch = False
            self.score_3_face = 1
            self.score_3_right_branch = True
            self.auto_position_source = 3
            self.poseEstimator.poseEst.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, FieldConstants.fieldWidth - 1.372, Rotation2d.fromDegrees(270))))
            self.poseEstimator.poseEstSingleTag.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, FieldConstants.fieldWidth - 1.372, Rotation2d.fromDegrees(270))))
            for idx, command in enumerate(self.p_for_3p_f1):
                self.p_for_3p_f1[idx] = self.flip_path_cmd_across_x(command)

    def flip_3_piece_to_f5(self, left_side : bool):
        if not left_side:
            self.score_1_face = 5
            self.score_1_right_branch = False
            self.score_2_face = 6
            self.score_2_right_branch = True
            self.score_3_face = 6
            self.score_3_right_branch = False
            self.left_source_auto = False
            self.auto_position_source = 3
            self.poseEstimator.poseEst.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, 1.372, Rotation2d.fromDegrees(270))))
            self.poseEstimator.poseEstSingleTag.resetPose(FieldConstants.flip_Pose2d(Pose2d(7.167, 1.372, Rotation2d.fromDegrees(270))))
            
        else:
            self.score_1_face = 3
            self.score_1_right_branch = True
            self.score_2_face = 2
            self.score_2_right_branch = False
            self.score_3_face = 2
            self.score_3_right_branch = True
            self.left_source_auto = True
            for idx, command in enumerate(self.autoroutines.p_for_f5):
                #     #mirror across x axis
                # print("pathplanner command ", idx + 1)
                self.autoroutines.p_for_f5[idx] = self.flip_path_cmd_across_x(command)

    def followPathCommand(self, pathName: str, pathConstraints=None):
        path = PathPlannerPath.fromPathFile(pathName)
        if pathConstraints != None:
            path._globalConstraints = pathConstraints
        return FollowPathCommand(
            path,
            self.drivetrain.get_pose, # Robot pose supplier
            self.drivetrain.get_robot_relative_speeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.drivetrain.drive_robot_relative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
            PPHolonomicDriveController(  # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(
                    const.X_KP, const.X_KI, const.X_KD
                ),  # Translation PID constants
                PIDConstants(
                    const.THETA_KP, const.THETA_KI, const.THETA_KD
                ),  # Rotation PID constants
            ),
            self.pathplanner_config, # The robot configuration
            self.drivetrain.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self.drivetrain # Reference to this subsystem to set requirements
        )



    ### DISABLED ###

    def disabled_mode(self):
        self.scheduler.cancelAll()
        # self.drivetrain.gyro_offset = self.drivetrain.gyro.get_roll()

        for subsystem in self.subsystems:
            subsystem.stop()

        while True:  # Needs to continuously call while robot is disabled.
            yield

    ### AUTONOMOUS ###
    def autonomous_mode(self):
        if FieldConstants.shouldFlip:
            desired_yaw = 270.0
        else:
            desired_yaw = 90.0
        self.poseEstimator.set_yaw(desired_yaw)
        self.has_coral = True # Start with preloaded coral
        self.scheduler.cancelAll()
        self.in_autonomous_mode = True

        if FieldConstants.shouldFlip:
            self.poseEstimator.set_yaw(90)
        else:
            self.poseEstimator.set_yaw(270)

        self.scheduler.schedule(self.auto)

    ### TELEOPERATED ###
    def teleop_mode(self):
        self.scheduler.cancelAll()
        # self.mechanisms_at_default = True
        self.funnel_intake.is_intaking = False
        self.end_effector.is_intaking = False
        self.running_pid_lineup = False
        self.score_intent = False
        self.in_autonomous_mode = False
        self.funnel_intake.piece_passing_through = False
        self.oi.robot_oriented_angle = self.poseEstimator.getYaw().degrees()

        while True:
            yield

    ### WAIT FUNCTION ###
    def wait(self, time):
        timer = Timer()
        timer.start()
        while not timer.hasElapsed(time):
            yield

    def log(self):
        """
        Logs some info to shuffleboard, and standard output
        """
        wpilib.SmartDashboard.putNumber("Score state", self.score_state.number)
        wpilib.SmartDashboard.putBoolean("Score piece", self.score_piece)
        wpilib.SmartDashboard.putBoolean("Mechanisms at default", self.mechanisms_at_default)
        wpilib.SmartDashboard.putBoolean("At scoring position", self.at_scoring_position)
        wpilib.SmartDashboard.putBoolean("OI Score Intent", self.score_intent)
        wpilib.SmartDashboard.putBoolean("Has Coral", self.has_coral)

        for s in self.subsystems:
            s.log()

        self.match_time = self.driverstation.getMatchTime()
        wpilib.SmartDashboard.putNumber("Match Time", self.match_time)
        wpilib.SmartDashboard.putNumber(
            "robot oriented angle", self.oi.robot_oriented_angle
        )


### MAIN ###

if __name__ == "__main__":
    # wpilib.run(Robot) go to \Robot-2023\robot folder and run "py -m robotpy run"
    pass
