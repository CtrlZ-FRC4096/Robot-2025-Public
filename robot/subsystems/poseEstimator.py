import time
import math
import numpy as np

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
    SwerveModuleState,
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

        # bl, fl, br, fr

        # fl, fr, bl, br

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

        self.curEstPose = Pose2d(0, 0, self.getYaw())
        # self.lastPeriodicEstPose = self.curEstPose

        self.poseEst = SwerveDrive4PoseEstimator(
            const.SWERVE_KINEMATICS, self.getYaw(), self.get_module_positions(), self.curEstPose  # type: ignore
        )

        # self.poseEst.setVisionMeasurementStdDevs((0.0001, 0.0001, 0.5))
        self.xystd = 0.3
        self.thetastd = 10.0  # .15

        # test position of camera 1 on front right module
        ROBOT_TO_CAM1 = Transform3d(
            Translation3d(-0.290, -0.295, 0.1699),  # X  # Y  # Z
            Rotation3d(
                0.0, np.deg2rad(-10.0), np.deg2rad(20.0 - 90)
            ),  # Roll  # Pitch  # Yaw
        )

        # Update with positionon robot
        ROBOT_TO_CAM2 = Transform3d(
            Translation3d(0.290, -0.295, 0.1699),  # X  # Y  # Z
            Rotation3d(
                0.0, np.deg2rad(-10.0), np.deg2rad(-20.0 - 90)
            ),  # Roll  # Pitch  # Yaw
        )

        # # Update with positionon robot
        # ROBOT_TO_CAM3 = Transform3d(
        #     Translation3d(-0.2764, 0.2805, 0.2949),  # X  # Y  # Z
        #     Rotation3d(
        #         0.0, 31.32 * (math.pi / 180), 180.0 * (math.pi / 180)
        #     ),  # Roll  # Pitch  # Yaw
        # )

        # # Update with positionon robot
        # ROBOT_TO_CAM4 = Transform3d(
        #     Translation3d(0.045, -0.286, 0.588),  # X  # Y  # Z
        #     Rotation3d(
        #         0 * (math.pi / 180),
        #         -10.0 * (math.pi / 180),
        #         -24.62 * (math.pi / 180),
        #     ),  # Roll  # Pitch  # Yaw
        # )

        self.cams = [
            WrapperedPhotonCamera("camera_1", ROBOT_TO_CAM1),
            WrapperedPhotonCamera("camera_2", ROBOT_TO_CAM2),
            # WrapperedPhotonCamera("Camera3", ROBOT_TO_CAM3),
            # WrapperedPhotonCamera("Camera4", ROBOT_TO_CAM4),
        ]

        self.poseConverge = True
        self.isFirstTick = True
        self.last_periodic_accel_x = 0
        self.last_periodic_accel_y = 0

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

    def swerve_state_to_vel_vector(self, swerve_module_state: SwerveModuleState):
        return Translation2d(swerve_module_state.speed, swerve_module_state.angle)

    def is_moving(self):
        swerve_chassis = const.SWERVE_KINEMATICS.toChassisSpeeds(
            self.get_module_states()
        )
        return swerve_chassis.vx > 0.01 or swerve_chassis.vy > 0.01

    def get_skidding_ratio(self):
        if not (self.is_moving()):
            return 1  # is this ok?
        swerve_module_states = self.get_module_states()
        self.angular_velocity = const.SWERVE_KINEMATICS.toChassisSpeeds(
            swerve_module_states
        ).omega
        self.swerve_state_rotations = const.SWERVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds(0, 0, self.angular_velocity)
        )
        self.swerve_states_translation_magnitudes = []

        for idx in range(len(swerve_module_states)):
            swerve_state_vector = self.swerve_state_to_vel_vector(
                swerve_module_states[idx]
            )
            swerve_state_rotation_vector = self.swerve_state_to_vel_vector(
                self.swerve_state_rotations[idx]
            )
            self.swerve_states_translation_magnitudes.append(
                (swerve_state_vector - swerve_state_rotation_vector).norm()
            )
        self.max_trans_speed = max(self.swerve_states_translation_magnitudes)
        self.min_trans_speed = min(self.swerve_states_translation_magnitudes)

        return self.max_trans_speed / self.min_trans_speed

    def get_jerk_val(self):
        cur_accel_x = self.gyro.get_acceleration_x().value
        cur_accel_y = self.gyro.get_acceleration_y().value

        cur_jerk_x = abs(cur_accel_x - self.last_periodic_accel_x) / 0.05
        cur_jerk_y = abs(cur_accel_y - self.last_periodic_accel_y) / 0.05

        self.last_period_accel_x = cur_accel_x
        self.last_period_accel_y = cur_accel_x

        return np.sqrt(cur_jerk_x**2 + cur_jerk_y**2)

    def poseIsOffField(self, pose: Pose2d):
        trans = pose.translation()
        x = trans.X()
        y = trans.Y()
        inY = -0.5 < y < FieldConstants.fieldWidth + 0.5
        inX = -0.5 < x < FieldConstants.fieldLength + 0.5
        return not (inX and inY)

    def candidate_pose_OK(self, candidate_pose: Pose2d):
        if self.poseIsOffField(candidate_pose):  # Check if the robot is on the field
            return False
        elif (
            self.get_skidding_ratio() > const.SKIDDING_RATIO_MAX
        ):  # TODO: Tune this in shop
            return False
        elif self.get_jerk_val() > const.COLLISION_JERK_MAX:
            return False
        # add more elifs as conditions
        else:
            return True
        
    def calculate_closest_reef_tag(self, relevant_tags):
        min_distance_to_tag = math.inf
        closest_reef_tag = None
        for tag in relevant_tags:
            distance = math.sqrt((self.curEstPose.X()-tag.X())**2 + (self.curEstPose.Y() - tag.Y())**2)
            if distance < min_distance_to_tag:
                min_distance_to_tag = distance
                closest_reef_tag = tag
        return closest_reef_tag

    def periodic(self):
        allianceColor = DriverStation.getAlliance()
        single_tag_IDs = set()
        single_tag_poses = []

        for idx, cam in enumerate(self.cams):
            cam.update(self.curEstPose, allianceColor=allianceColor)

            observations = cam.getPoseEstimates()
            tags = cam.getTagPositions()
            single_tag_poses.append(cam.getPoseSingleTag())
            single_tag_IDs.add(cam.getSingleTagIDs())
            #filter by closest based on global pose
            relevant_tags = single_tag_IDs.intersection(FieldConstants.reef_tags)
            closest_reef_tag = self.calculate_closest_reef_tag(relevant_tags)


            tag_dist = 0.0
            theta_modifier = 1.0
            xy_modifier = 1.0

            for tag in tags:
                tag2D = tag.toPose2d()
                tag_dist += (self.curEstPose - tag2D).translation().norm()
            if len(tags) > 0:
                tag_dist /= len(tags)
            if len(tags) == 1:
                theta_modifier = 1000.0
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
                        * xy_modifier,  # * (min_ambiguity / 0.4),
                        self.xystd
                        * (tag_dist**2)
                        * xy_modifier,  # * (min_ambiguity / 0.4),
                        self.thetastd
                        * (tag_dist**2)
                        * theta_modifier,  # * (min_ambiguity / 0.4),
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

        

        # if self.isFirstTick:
        #     self.isFirstTick = False

        self.poseEst.update(self.getYaw(), self.get_module_positions())
        # self.lastPeriodicEstPose = self.curEstPose

        SmartDashboard.putNumber("skidding ratio", self.get_skidding_ratio())
        SmartDashboard.putNumber("jerk val", self.get_jerk_val())

        possible_pose = self.poseEst.getEstimatedPosition()

        SmartDashboard.putBoolean("pose 4 u :3", self.candidate_pose_OK(possible_pose))

        if self.candidate_pose_OK(possible_pose):
            self.curEstPose = self.poseEst.getEstimatedPosition()

        if (self.robot.leds.mode == self.robot.leds.MODE_LOST_ODOMETRY) or (
            self.robot.leds.mode == self.robot.leds.MODE_ODOMETRY
        ):
            if not self.poseConverge:
                self.robot.leds.set_mode(self.robot.leds.MODE_LOST_ODOMETRY)
            elif self.poseConverge:
                self.robot.leds.set_mode(self.robot.leds.MODE_ODOMETRY)

        SmartDashboard.putData("Field", self.field)
        #self.field.setRobotPose(self.poseEst.getEstimatedPosition())
        self.field.setRobotPose(Pose2d(single_tag_poses[0].translation(), self.gyro.get_yaw()))

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
