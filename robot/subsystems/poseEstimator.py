import time
import math

from collections import deque

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot

from phoenix6.hardware import Pigeon2, TalonFX
from wpilib import (
    DriverStation,
    SmartDashboard,
    Timer,
    Field2d,
    AnalogAccelerometer,
    BuiltInAccelerometer,
)
from wpimath.geometry import (
    Pose2d,
    Pose3d,
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

# from leds import LEDs
# from shooter import Shooter

# from pathplannerlib.path import PathConstraints

# from commands2 import SubsystemBase
from wpilibextra.coroutine.subsystem import Subsystem
from swerve.swervemodule import SwerveModule
from wpimath.controller import PIDController
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.config import (
    # HolonomicPathFollowerConfig,
    # ReplanningConfig,
    PIDConstants,
)

from wpilib import BuiltInAccelerometer
from wpimath.filter import LinearFilter

from pathplannerlib.path import PathPlannerTrajectory
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpimath.estimator import SwerveDrive4PoseEstimator
from photoncamera import WrapperedPhotonCamera
from wpimath.units import degreesToRadians


class PoseEstimator(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot

        self.gyro = Pigeon2(const.SWERVE_PIGEON_ID, "carnivore")

        self.gyro_offset = 0.0
        self.gyro.set_yaw(self.gyro_offset)

        self.field = Field2d()

        self.modules = (
            SwerveModule(
                "front_left",
                const.SWERVE_ANGLE_OFFSET_FRONT_LEFT,
                const.SWERVE_DRIVE_MOTOR_ID_FRONT_LEFT,
                const.SWERVE_ANGLE_MOTOR_ID_FRONT_LEFT,
                const.SWERVE_CANCODER_ID_FRONT_LEFT,
                const.SWERVE_DRIVE_INVERT_FRONT_LEFT,
                const.SWERVE_ANGLE_INVERT_FRONT_LEFT,
            ),
            SwerveModule(
                "front_right",
                const.SWERVE_ANGLE_OFFSET_FRONT_RIGHT,
                const.SWERVE_DRIVE_MOTOR_ID_FRONT_RIGHT,
                const.SWERVE_ANGLE_MOTOR_ID_FRONT_RIGHT,
                const.SWERVE_CANCODER_ID_FRONT_RIGHT,
                const.SWERVE_DRIVE_INVERT_FRONT_RIGHT,
                const.SWERVE_ANGLE_INVERT_FRONT_RIGHT,
            ),
            SwerveModule(
                "back_left",
                const.SWERVE_ANGLE_OFFSET_BACK_LEFT,
                const.SWERVE_DRIVE_MOTOR_ID_BACK_LEFT,
                const.SWERVE_ANGLE_MOTOR_ID_BACK_LEFT,
                const.SWERVE_CANCODER_ID_BACK_LEFT,
                const.SWERVE_DRIVE_INVERT_BACK_LEFT,
                const.SWERVE_ANGLE_INVERT_BACK_LEFT,
            ),
            SwerveModule(
                "back_right",
                const.SWERVE_ANGLE_OFFSET_BACK_RIGHT,
                const.SWERVE_DRIVE_MOTOR_ID_BACK_RIGHT,
                const.SWERVE_ANGLE_MOTOR_ID_BACK_RIGHT,
                const.SWERVE_CANCODER_ID_BACK_RIGHT,
                const.SWERVE_DRIVE_INVERT_BACK_RIGHT,
                const.SWERVE_ANGLE_INVERT_BACK_RIGHT,
            ),
        )

        # This is to give CANCoders time to settle. Normally sleep calls are very bad but they're okay
        # here in the constructor/init.
        time.sleep(1.0)
        self.reset_modules_to_absolute()

        self.odometry = SwerveDrive4Odometry(
            const.SWERVE_KINEMATICS, self.getYaw(), self.get_module_positions()  # type: ignore
        )

        self.curEstPose = Pose2d()

        self.speaker_angle = 0

        self.poseEst = SwerveDrive4PoseEstimator(
            const.SWERVE_KINEMATICS, self.getYaw(), self.get_module_positions(), self.curEstPose  # type: ignore
        )

        # self.poseEst.setVisionMeasurementStdDevs((0.0001, 0.0001, 0.5))
        self.xystd = 0.1  # .1
        self.thetastd = 0.15  # .15

        # Update with position on robot
        ROBOT_TO_CAM1 = Transform3d(
            Translation3d(-0.1570, -0.2819, 0.2023),  # X  # Y  # Z
            Rotation3d(
                0.0, 30.0 * (math.pi / 180), 180.0 * (math.pi / 180)
            ),  # Roll  # Pitch  # Yaw
        )

        # Update with positionon robot
        ROBOT_TO_CAM2 = Transform3d(
            Translation3d(
                0.06, 0.286, 0.423
            ),  # X  # Y  # Z # .0692 for super structure
            Rotation3d(
                0 * (math.pi / 180),
                -10.0 * (math.pi / 180),
                24.62 * (math.pi / 180),
            ),  # Roll  # Pitch  # Yaw
        )

        # Update with positionon robot
        ROBOT_TO_CAM3 = Transform3d(
            Translation3d(-0.2764, 0.2805, 0.2949),  # X  # Y  # Z
            Rotation3d(
                0.0, 31.32 * (math.pi / 180), 180.0 * (math.pi / 180)
            ),  # Roll  # Pitch  # Yaw
        )

        # Update with positionon robot
        ROBOT_TO_CAM4 = Transform3d(
            Translation3d(0.045, -0.286, 0.588),  # X  # Y  # Z
            Rotation3d(
                0 * (math.pi / 180),
                -10.0 * (math.pi / 180),
                -24.62 * (math.pi / 180),
            ),  # Roll  # Pitch  # Yaw
        )

        self.cams = [
            WrapperedPhotonCamera("Camera1", ROBOT_TO_CAM1),
            WrapperedPhotonCamera("Camera2", ROBOT_TO_CAM2),
            WrapperedPhotonCamera("Camera3", ROBOT_TO_CAM3),
            WrapperedPhotonCamera("Camera4", ROBOT_TO_CAM4),
        ]

        self.poseConverge = True

    def stop(self):
        print("sike this aint stoppin")

    def set_module_states(self, desired_states):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desired_states, const.SWERVE_MAX_SPEED
        )

        for idx, module in enumerate(self.modules):
            module.set_desired_state(desired_states[idx], False)

    def get_module_states(self):
        return tuple([module.get_state() for module in self.modules])

    def get_module_positions(self):
        return tuple([module.get_position() for module in self.modules])

    def zero_gyro(self):
        self.set_yaw(0)

    def set_yaw(self, yaw):
        SmartDashboard.putNumber("Gyro/Set Yaw", yaw)
        self.gyro.set_yaw(yaw)

    def getYaw(self):
        if const.SWERVE_INVERT_GYRO:
            return Rotation2d.fromDegrees((360 - self.gyro.get_yaw().value) % 360)
        else:
            return Rotation2d.fromDegrees(self.gyro.get_yaw().value % 360)

    def reset_modules_to_absolute(self):
        for module in self.modules:
            module.reset_to_absolute()

    def periodic(self):
        allianceColor = DriverStation.getAlliance()

        for idx, cam in enumerate(self.cams):
            if (
                cam.cam.getName() == "Camera1"
            ):  # Change this to name of camera facing april tag on reef
                pass

            cam.update(self.curEstPose, allianceColor=allianceColor)

            observations = cam.getPoseEstimates()
            tags = cam.getTagPositions()

            tag_dist = 0.0
            min_ambiguity = 10.0
            theta_modifier = 1.0
            xy_modifier = 1.0
            auto_modifier = 1.0

            for ambig in cam.getTagAmbiguity():
                if ambig < min_ambiguity:
                    min_ambiguity = ambig

            for tag in tags:
                tag2D = tag.toPose2d()
                tag_dist += (self.curEstPose - tag2D).translation().norm()
            if len(tags) > 0:
                tag_dist /= len(tags)
            if len(tags) == 1:
                theta_modifier = 1000.0
                if min_ambiguity > 0.1:
                    xy_modifier = 3.0
            if self.robot.in_autonomous_mode:
                auto_modifier = 3.0
            if tag_dist > 4:  # if the robot is more than 4 meters away from the target
                xy_modifier = 3.0
                theta_modifier = 3.0

            # print(tag_dist)
            for observation in observations:
                self.poseEst.addVisionMeasurement(
                    observation.estFieldPose,
                    observation.time,
                    (
                        self.xystd
                        * (tag_dist**2)
                        * xy_modifier
                        * auto_modifier,  # * (min_ambiguity / 0.4),
                        self.xystd
                        * (tag_dist**2)
                        * xy_modifier
                        * auto_modifier,  # * (min_ambiguity / 0.4),
                        self.thetastd
                        * (tag_dist**2)
                        * theta_modifier
                        * auto_modifier,  # * (min_ambiguity / 0.4),
                    ),
                )
                if (
                    observation.estFieldPose - self.poseEst.getEstimatedPosition()
                ).translation().norm() <= 0.5:
                    self.poseConverge = True
                else:
                    self.poseConverge = False
                self.camTargetsVisible = True
            # self.telemetry.addVisionObservations(observations) #Might need later https://github.com/RobotCasserole1736/RobotCasserole2024/blob/fa033322e6f4efe87e8b1af938d8a3f69599f29b/drivetrain/poseEstimation/drivetrainPoseTelemetry.py#L15

        self.poseEst.update(self.getYaw(), self.get_module_positions())
        self.curEstPose = self.poseEst.getEstimatedPosition()
        candidate_pose = self.poseEst.getEstimatedPosition()

        if (
            (candidate_pose.x > -0.5)
            and (candidate_pose.x < self.robot.fieldConstants.fieldLength + 0.5)
            and (candidate_pose.y > -0.5)
            and (candidate_pose.y < self.robot.fieldConstants.fieldWidth + 0.5)
        ):  # Check if the robot is on the field
            self.curEstPose = candidate_pose

        if (self.robot.leds.mode == self.robot.leds.MODE_LOST_ODOMETRY) or (
            self.robot.leds.mode == self.robot.leds.MODE_ODOMETRY
        ):
            if not self.poseConverge:
                self.robot.leds.set_mode(self.robot.leds.MODE_LOST_ODOMETRY)
            elif self.poseConverge:
                self.robot.leds.set_mode(self.robot.leds.MODE_ODOMETRY)

        SmartDashboard.putData("Field", self.field)
        self.field.setRobotPose(self.poseEst.getEstimatedPosition())

        SmartDashboard.putNumber("Camera/Odometry X", self.curEstPose.x)
        SmartDashboard.putNumber("Camera/Odometry Y", self.curEstPose.y)
        SmartDashboard.putNumber(
            "Camera/Odometry Theta", self.curEstPose.rotation().degrees()
        )

        SmartDashboard.putNumber(
            "Swerve/Odometry X", self.odometry.getPose().x_feet * 0.305
        )
        SmartDashboard.putNumber(
            "Swerve/Odometry Y", self.odometry.getPose().y_feet * 0.305
        )
        SmartDashboard.putNumber(
            "Swerve/Odometry Theta", self.odometry.getPose().rotation().degrees()
        )
        SmartDashboard.putNumber("Gyro/Yaw", self.getYaw().degrees())
        # SmartDashboard.putNumber("Gyro/Roll", self.roll)

        self.odometry.update(self.getYaw(), self.get_module_positions())

        for module in self.modules:
            SmartDashboard.putNumber(f"Swerve/{module.module_name}/Cancoder Angle", module.get_angle_CANcoder().degrees())  # type: ignore
            SmartDashboard.putNumber(f"Swerve/{module.module_name}/Motor Angle", module.get_position().angle.degrees())  # type: ignore
            SmartDashboard.putNumber(
                f"Swerve/{module.module_name}/Velcoity", module.get_state().speed
            )

    def log(self):
        pass

