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
import subsystems.drivetrain

# import subsystems.limelight
import subsystems.leds

import subsystems.limelight
import subsystems.poseEstimator

from wpilibextra.coroutine.coroutine_robot import CoroutineRobot
from wpilibextra.remote_shell import RemoteShell
from coroutines import Coroutines

import inspect
import autoroutines

from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController

from wpimath.geometry import Rotation2d

from field_const import FieldConstants


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
        const.IS_SIMULATION = self.isSimulation()
        self.fieldConstants = FieldConstants()

        self.has_note = False

        # Command scheduler
        self.scheduler = CommandScheduler.getInstance()

        # subsystems
        self.drivetrain = subsystems.drivetrain.Drivetrain(self)
        self.leds = subsystems.leds.LEDs(self)
        self.limelight = subsystems.limelight.Limelight_Wrapper()
        self.poseEstimator = subsystems.poseEstimator.PoseEstimator(self)

        self.subsystems = [
            self.drivetrain,
            self.leds,
            self.poseEstimator,
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

        self.match_time = -1
        ### FIELD LOGGING ###
        self.field = Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)
        ### LOGGING ###
        self.remote_shell = RemoteShell(self)

        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

        @self.addPeriodic(period=0.25, offset=0)
        def _():
            self.log()
            pass

        @self.addPeriodic(period=0.05, offset=-0.01)
        def _leds():
            self.leds.periodicX()
            pass

        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.drivetrain.get_pose,  # Robot pose supplier
            self.drivetrain.reset_odometry,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.drivetrain.get_robot_relative_speeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.drivetrain.drive_robot_relative(
                speeds
            ),  # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController(  # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(
                    const.X_KP, const.X_KI, const.X_KD
                ),  # Translation PID constants
                PIDConstants(
                    const.THETA_KP, const.THETA_KI, const.THETA_KD
                ),  # Rotation PID constants
            ),
            config,  # The robot configuration
            self.drivetrain.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self.drivetrain,  # Reference to this subsystem to set requirements
        )

        ## Need to change this and redeloy
        self.path = PathPlannerAuto("Test Auto")  # centerline steal 2nd match

        self.in_autonomous_mode = False

        while True:
            yield
            self.scheduler.run()

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
        self.scheduler.cancelAll()
        self.in_autonomous_mode = True
        self.scheduler.schedule(self.path.schedule())

        # self.scheduler.schedule(self.auto_chooser.getSelected()(self))

    ### TELEOPERATED ###
    def teleop_mode(self):
        self.leds.set_mode(self.leds.MODE_ODOMETRY)
        self.scheduler.cancelAll()
        self.in_autonomous_mode = False

        while True:
            yield

    ### MISC ###

    def stop_all_subsystems(self):  # Update as we add more subsystems
        self.funnel_intake.is_running = False

    def log(self):
        """
        Logs some info to shuffleboard, and standard output
        """
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
