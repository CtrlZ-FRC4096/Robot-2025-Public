from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot


import time
import math
from phoenix6.hardware import Pigeon2, TalonFX
from wpilib import DriverStation, SmartDashboard, Timer, Field2d
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
    Translation3d,
    Transform3d,
    Rotation3d,
)
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
)
from phoenix6 import configs


# from pathplannerlib.commands import PathfindHolonomic


import const
from field_const import FieldConstants

# from leds import LEDs
# from shooter import Shooter

# from pathplannerlib.path import PathConstraints

# from commands2 import SubsystemBase
from wpilibextra.coroutine.subsystem import Subsystem
from swerve.swervemodule import SwerveModule
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.config import PIDConstants

from pathplannerlib.path import PathPlannerTrajectory
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpimath.estimator import SwerveDrive4PoseEstimator
from photoncamera import WrapperedPhotonCamera
from wpimath.units import degreesToRadians, inchesToMeters


class Drivetrain(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot

        self.angle_pid = PIDController(0.075, 0.0, 0.001)
        self.angle_pid.enableContinuousInput(0, 360)
        self.angle_pid.setTolerance(0.5)  # Set position tolerance to 0.5 degrees

        self.x_controller = PIDController(1.85, 0.0, 0.0)
        self.y_controller = PIDController(1.85, 0.0, 0.0)
        self.theta_controller = PIDController(0.12, 0.0, 0.001)

        ## Need to check these tolerances
        self.x_controller.setTolerance(0.05, 0.01)
        self.y_controller.setTolerance(0.05, 0.01)
        self.theta_controller.enableContinuousInput(0, 360)
        self.theta_controller.setTolerance(1.0, 0.01)

        ### Field Visualisation - Needs testing ###
        self.previous_chassisspeeds = ChassisSpeeds()
        # self.curPose = Pose2d(inchesToMeters(235.726), 0.8, Rotation2d.fromDegrees(0))
        # self.isFirstTick = True

    def drive(self, translation: Translation2d, rotation, field_relative, is_open_loop):
        SmartDashboard.putNumber("Swerve/Translation X", translation.x)
        SmartDashboard.putNumber("Swerve/Translation Y", translation.y)
        SmartDashboard.putNumber("Swerve/Rotation", rotation)
        SmartDashboard.putBoolean("Swerve/With PID", False)
        if field_relative:
            module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.x,
                    translation.y,
                    -rotation,
                    self.robot.poseEstimator.getYaw(),
                )
            )
        else:  # Robot relative
            module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds(
                    translation.x,
                    translation.y,
                    rotation,
                )
            )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            module_states, const.SWERVE_MAX_SPEED
        )

        for idx, module in enumerate(self.robot.poseEstimator.modules):
            module.set_desired_state(module_states[idx], is_open_loop)

        # self.curPose = Pose2d(self.curPose.X() + min(translation.X(), (3.0 * translation.X()) / abs(translation.X()) if translation.X() != 0 else 3.0), self.curPose.Y() + min(translation.Y(), (3.0 * translation.Y()) / abs(translation.Y()) if translation.Y() != 0 else 3.0), Rotation2d.fromDegrees(0))
        # print(self.curPose)
        # pose = self.robot.poseEstimator.field.getObject("current pose")

        # pose.setPose(self.curPose)


    def drive_with_pid(self, translation: Translation2d, target_angle):
        pid_output = self.angle_pid.calculate(self.robot.poseEstimator.getYaw().degrees(), target_angle)  # type: ignore

        if self.angle_pid.atSetpoint():
            pid_output = 0

        # if not in_motion:
        #     pid_output += math.copysign(0.2, pid_output)
        SmartDashboard.putBoolean("Swerve/With PID", True)
        self.drive(
            translation, pid_output, True, False
        )  # change is_open_loop back to False once done w/ driver tests
        # print(in_motion)

    def drive_robot_relative(
        self, chassis_speeds: ChassisSpeeds
    ):  # only use for pathplannerlib
        chassis_speeds.omega = -chassis_speeds.omega
        module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(chassis_speeds)

        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            module_states, const.SWERVE_MAX_SPEED
        )

        for idx, module in enumerate(self.robot.poseEstimator.modules):
            # print(module_states[idx].speed)
            module.set_desired_state(module_states[idx], is_open_loop=False)

    def go_to_pose_profiled_pid(self, target_pose : Translation2d):

        current_pose = self.robot.poseEstimator.curEstPose

        # Calculate the control outputs
        vx = self.x_controller.calculate(current_pose.X(), target_pose.X()) # meters / 0.05 seconds
        vy = self.y_controller.calculate(current_pose.Y(), target_pose.Y())
        omega = self.theta_controller.calculate(
            current_pose.rotation().degrees(), target_pose.rotation().degrees()
        )


        # Check if the controllers are at their setpoints
        if (
            self.x_controller.atSetpoint()
            and self.y_controller.atSetpoint()
            and self.theta_controller.atSetpoint()
        ):
            self.robot.oi.running_pid_lineup = False
            self.robot.at_scoring_position = True
            self.stop()
            return
            # Optionally, stop the drivetrain if at setpoint


        # Drive the robot using the calculated velocities
        self.drive(Translation2d(vx, vy), omega, True, False)

        # Update SmartDashboard values for debugging
        SmartDashboard.putNumber("t_pose x", target_pose.X())
        SmartDashboard.putNumber("t_pose y", target_pose.Y())
        SmartDashboard.putNumber("vx", vx)
        SmartDashboard.putNumber("vy", vy)
        SmartDashboard.putNumber("omega", 0)

    def stop(self):
        self.drive(Translation2d(0, 0), 0, False, True)

    def get_pose(self):
        return self.robot.poseEstimator.curEstPose

    def reset_odometry(self, pose):
        self.robot.poseEstimator.odometry.resetPosition(self.robot.poseEstimator.getYaw(), [*self.robot.poseEstimator.get_module_positions()], pose)  # type: ignore
        self.robot.poseEstimator.poseEst.resetPosition(
            self.robot.poseEstimator.getYaw(),
            [*self.robot.poseEstimator.get_module_positions()],
            pose,
        )

    def get_robot_relative_speeds(self):
        module_states = (
            self.robot.poseEstimator.get_module_states()
        )  # Check this in swervemodule.py, we need to convert kraken speed to m/s
        chassis_speeds = const.SWERVE_KINEMATICS.toChassisSpeeds(module_states)  # type: ignore
        return chassis_speeds

    def shouldFlipPath(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def periodic(self):
        # if DriverStation.isDisabled():
        #     self.reset_modules_to_absolute()

        SmartDashboard.putData("PID Controller for going to reef, x", self.x_controller)
        SmartDashboard.putData("PID Controller for going to reef, y", self.y_controller)
        SmartDashboard.putData(
            "PID Controller for going to reef, theta", self.theta_controller
        )

        SmartDashboard.putData("PID Controller (Drivetrain)", self.angle_pid)
        SmartDashboard.putBoolean("Angle at Setpoint", self.angle_pid.atSetpoint())
        SmartDashboard.putNumber("PID Controller Error", self.angle_pid.getError())

    def log(self):
        pass
