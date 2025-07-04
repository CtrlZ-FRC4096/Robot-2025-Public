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
    SwerveModuleState
)
from phoenix6 import configs
# from shapely import Polygon, Point


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
from wpimath.units import degreesToRadians, inchesToMeters, radiansToDegrees
from robot_scoring_positions import RobotScoringPositions
from collections import deque

class Drivetrain(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot


        self.counter = 1

        self.angle_pid = PIDController(0.075, 0.0, 0.001)
        self.angle_pid.enableContinuousInput(0, 360)
        self.angle_pid.setTolerance(0.5)  # Set position tolerance to 0.5 degrees

        self.x_controller = PIDController(2.0, 0.01, 0.025) #0.01
        self.y_controller = PIDController(2.0, 0.01, 0.025) #0.01
        self.xy_controller = ProfiledPIDController(2.0, 0.0, 0.025, TrapezoidProfile.Constraints(4.0, 4.0))
        self.theta_controller = PIDController(0.07, 0.01, 0.0015)
        
        self.inter_max_vel = 4.0
        self.inter_max_acc = 4.0
        constraints = TrapezoidProfile.Constraints(self.inter_max_vel, self.inter_max_acc)
        # self.xy_controller = ProfiledPIDController(2.0, 0.01, 0.025)#, constraints, period=0.05)
        self.xy_inter_controller = ProfiledPIDController(1.55, 0.0, 0.0, constraints)





        ## Need to check these tolerances
        self.x_controller.setTolerance(0.03, 0.1) #0.025, 0.1
        self.y_controller.setTolerance(0.03, 0.1) #0.025, 0.1
        self.xy_controller.setTolerance(0.0225, 0.15)
        self.theta_controller.enableContinuousInput(0, 360)
        self.theta_controller.setTolerance(2.8, 2.0) #3.0, 0.1
        self.xy_inter_controller.setTolerance(0.15, 2.0)

        self.at_inter_pose = False
        self.inter_pose = Pose2d()
        self.log_chassis = ChassisSpeeds()
        self.limit_reef_acc = False
        self.acc_limit_counter = 1
        self.xy_max_vel = 4.0
        self.xy_max_acc = 4.0
        ### Field Visualisation - Needs testing ###
        self.previous_chassisspeeds = ChassisSpeeds()
        # self.curPose = Pose2d(inchesToMeters(235.726), 0.8, Rotation2d.fromDegrees(0))
        # self.isFirstTick = True

        # self.scoring_position_length = 2
        # self.at_scoring_position_drivetrain = deque(maxlen=self.scoring_position_length)
        # for i in range(self.scoring_position_length):
        #     self.at_scoring_position_drivetrain.append(False)

        buffer = 0.47
        reefVertices = [
                self.robot.poseEstimator.get_path_to_reef(False, 1, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
                self.robot.poseEstimator.get_path_to_reef(False, 2, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
                self.robot.poseEstimator.get_path_to_reef(False, 3, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
                self.robot.poseEstimator.get_path_to_reef(False, 4, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
                self.robot.poseEstimator.get_path_to_reef(False, 5, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
                self.robot.poseEstimator.get_path_to_reef(False, 6, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
            ]#face's right branch point
        reefAngles = []
        for idx in range(6):
            # face: idx + 1
            angle_face = FieldConstants.Reef.centerFaces[idx].rotation().degrees()
            angle_face_minus_1 = FieldConstants.Reef.centerFaces[(idx - 1) % 6].rotation().degrees()
            delta_angle = (angle_face_minus_1 - angle_face) % 360
            if delta_angle > 180:
                delta_angle -= 360
            angle_mid = (angle_face + delta_angle / 2) % 360
            reefAngles.append(angle_mid)
        self.reefForInReef = []
        for idx in range(6):
            self.reefForInReef.append(Translation2d(reefVertices[idx].X() + (buffer * math.cos(degreesToRadians(reefAngles[idx]))), reefVertices[idx].Y() + (buffer * math.sin(degreesToRadians(reefAngles[idx])))))
        
        self.coral_blocking_length = 5
        self.coral_blocking = deque(maxlen=self.coral_blocking_length)
        for i in range(self.coral_blocking_length):
            self.coral_blocking.append(False)

    def drive(self, translation: Translation2d, rotation, field_relative, is_open_loop):
        SmartDashboard.putNumber("Swerve/Translation X", translation.x)
        SmartDashboard.putNumber("Swerve/Translation Y", translation.y)
        SmartDashboard.putNumber("Swerve/Rotation", rotation)
        SmartDashboard.putBoolean("Swerve/With PID", False)
        
        if field_relative and not self.robot.isSimulation():
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
        if self.robot.in_autonomous_mode:
            max_speed = 3.4
        else:
            max_speed = const.SWERVE_MAX_SPEED
        module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
                module_states, max_speed
            )
        self.log_chassis = const.SWERVE_KINEMATICS.toChassisSpeeds(module_states)
        SmartDashboard.putNumber("translation x", translation.x / 20)
        SmartDashboard.putNumber("translation y", translation.y / 20)
        SmartDashboard.putNumber("translation omega", radiansToDegrees(rotation) / 20)

        SmartDashboard.putNumber("chassis log vx", self.log_chassis.vx / 45)
        SmartDashboard.putNumber("chassis log vy", self.log_chassis.vy / 45)
        SmartDashboard.putNumber("chassis log omega dps", self.log_chassis.omega_dps / 20)
        if self.robot.isSimulation():
            curPose = self.robot.poseEstimator.curEstPose
            # self.robot.poseEstimator.poseEstSingleTag.resetPose(Pose2d(curPose.X() + log_chassis.vx, curPose.Y() + log_chassis.vx, Rotation2d.fromDegrees(curPose.rotation().degrees() + log_chassis.omega_dps / 50)))
            # self.robot.poseEstimator.set_yaw(curPose.rotation().degrees() + log_chassis.omega_dps / 20)
            self.robot.poseEstimator.curEstPose = Pose2d(curPose.X() + self.log_chassis.vx / 45, curPose.Y() + self.log_chassis.vy / 45, Rotation2d.fromDegrees(curPose.rotation().degrees() + self.log_chassis.omega_dps / 20))
            if self.robot.poseEstimator.poseIsOffField(self.robot.poseEstimator.curEstPose) or self.in_reef(self.robot.poseEstimator.curEstPose.translation()):
                self.robot.poseEstimator.curEstPose = curPose
            self.robot.poseEstimator.set_yaw(self.robot.poseEstimator.curEstPose.rotation().degrees() + self.log_chassis.omega_dps / 20)
        else:
            for idx, module in enumerate(self.robot.poseEstimator.modules):
                SmartDashboard.putNumber("module state " + str(idx + 1), module_states[idx].speed)
                module.set_desired_state(module_states[idx], is_open_loop)

        # self.curPose = Pose2d(self.curPose.X() + min(translation.X(), (3.0 * translation.X()) / abs(translation.X()) if translation.X() != 0 else 3.0), self.curPose.Y() + min(translation.Y(), (3.0 * translation.Y()) / abs(translation.Y()) if translation.Y() != 0 else 3.0), Rotation2d.fromDegrees(0))
        # print(self.curPose)
        # pose = self.robot.poseEstimator.field.getObject("current pose")

        # pose.setPose(self.curPose)
    # def in_reef(self, pose: Translation2d):
    #         hexagon_points = [(self.reefForInReef[idx].X(), self.reefForInReef[idx].Y()) for idx in range(6)]
    #         hexagon = Polygon(hexagon_points)
    #         point = Point(pose.X(), pose.Y())
    #         return hexagon.contains(point)

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
        self, chassis_speeds: ChassisSpeeds, feedfoward=None
    ):  # only use for pathplannerlib
        chassis_speeds.omega = -chassis_speeds.omega
        module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(chassis_speeds)

        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            module_states, const.SWERVE_MAX_SPEED
        )

        for idx, module in enumerate(self.robot.poseEstimator.modules):
            # print(module_states[idx].speed)
            module.set_desired_state(module_states[idx], is_open_loop=False)

    def go_to_pose_profiled_pid(self, target_pose : Translation2d, feedforward_x=0.0, feedforward_y=0.0, feedfoward_theta=0.0):

        current_pose = self.robot.poseEstimator.curEstPose

        # Calculate the control outputs
        vx = self.x_controller.calculate(current_pose.X(), target_pose.X()) + feedforward_x # meters / 0.05 seconds
        vy = self.y_controller.calculate(current_pose.Y(), target_pose.Y()) + feedforward_y

        # if FieldConstants.shouldFlip:
        #     vx = -vx
        #     vy = -vy

        omega = self.theta_controller.calculate(
            current_pose.rotation().degrees(), target_pose.rotation().degrees()
        ) + feedfoward_theta

        # Check if the controllers are at their setpoints
        if (
            self.x_controller.atSetpoint()
            and self.y_controller.atSetpoint()
            and self.theta_controller.atSetpoint()
        ):
            # self.robot.running_pid_lineup = False
            if self.robot.score_intent and self.robot.running_pid_lineup:
                # self.at_scoring_position_drivetrain.appendleft(True)
                self.robot.at_scoring_position = True
            if self.robot.is_intaking and self.robot.running_pid_lineup:
                self.robot.at_intake_position = True
            # self.stop()
            # Optionally, stop the drivetrain if at setpoint
        # else:
        #     self.at_scoring_position_drivetrain.appendleft(False)


        # Drive the robot using the calculated velocities
        self.drive(Translation2d(vx, vy), omega, True, False)

        # Update SmartDashboard values for debugging
        SmartDashboard.putNumber("t_pose x", target_pose.X())
        SmartDashboard.putNumber("t_pose y", target_pose.Y())
        SmartDashboard.putNumber("vx", vx)
        SmartDashboard.putNumber("vy", vy)
        SmartDashboard.putNumber("omega", omega)

    def go_to_pose_angle_addition(self, final_pose : Pose2d, feedforward_x=0.0, feedforward_y=0.0, feedfoward_theta=0.0):
        cur_pose = self.robot.poseEstimator.curEstPose
        if self.robot.isSimulation():
            cur_speeds = self.log_chassis
        else:  
            cur_speeds = const.SWERVE_KINEMATICS.toChassisSpeeds(self.robot.poseEstimator.get_module_states())
        if self.at_inter_pose or cur_pose.translation().distance(final_pose.translation()) < 1.0:
            vel_angle = (final_pose.translation() - cur_pose.translation()).angle().radians()
            vel_mag = -1 * self.xy_controller.calculate(cur_pose.translation().distance(final_pose.translation()), 0) + 0.03
            
            vx = vel_mag * math.cos(vel_angle) + feedforward_x
            vy = vel_mag * math.sin(vel_angle) + feedforward_y
            omega = self.theta_controller.calculate(cur_pose.rotation().degrees(), final_pose.rotation().degrees()) + feedfoward_theta
            self.at_inter_pose = True
        else:
            dist_out = 0.4
            final_rot_out = degreesToRadians(final_pose.rotation().degrees() + 90)
            rot_proportion = dist_out / cur_pose.translation().distance(final_pose.translation())
            rot_diff = final_pose.rotation() - cur_pose.rotation()
            self.inter_pose = Pose2d(final_pose.X() + math.cos(final_rot_out) * dist_out, final_pose.Y() + math.sin(final_rot_out) * dist_out, final_pose.rotation())
            final_rot_in = Rotation2d(final_rot_out + math.pi)
            
            inter_delta = self.inter_pose.translation() - cur_pose.translation()
            inter_pose_angle = Rotation2d(math.atan2(inter_delta.y, inter_delta.x))

            alpha_angle = inter_pose_angle - final_rot_in
            velocity_angle = inter_pose_angle + alpha_angle #+ Rotation2d.fromDegrees(alpha_angle.degrees() ** 1/3)

            velocity = -1 * self.xy_inter_controller.calculate(self.inter_pose.translation().distance(cur_pose.translation()), 0)
            vx = velocity * math.cos(velocity_angle.radians()) + feedforward_x
            vy = velocity * math.sin(velocity_angle.radians()) + feedforward_y

            omega = self.theta_controller.calculate(
                cur_pose.rotation().degrees(), self.inter_pose.rotation().degrees()
            ) + feedfoward_theta

            if cur_pose.translation().distance(self.inter_pose.translation()) < 0.1:
                self.limit_reef_acc = True
                self.acc_limit_counter = 1
        
        if self.limit_reef_acc:
            if  self.acc_limit_counter < 15:
                self.inter_max_acc = 0.5
                self.xy_max_acc = 0.5
                self.acc_limit_counter += 1
            else:
                self.inter_max_acc = 4.0
                self.xy_max_acc = 4.0
        
        final_rotation_out = Rotation2d.fromDegrees(final_pose.rotation().degrees() + 90)
        coral_block_pose_x = math.cos(final_rotation_out.radians()) * inchesToMeters(4.0)
        coral_block_pose_y = math.sin(final_rotation_out.radians()) * inchesToMeters(4.0)
        coral_block_pose = Translation2d(final_pose.X() + coral_block_pose_x, final_pose.Y() + coral_block_pose_y)

        delta_pose = cur_pose.translation() - coral_block_pose
        vector = Translation2d(final_rotation_out.cos(), final_rotation_out.sin())
        projection_length = delta_pose.X() * vector.X() + delta_pose.Y() * vector.Y()
        closest_point = Translation2d(
            coral_block_pose.X() + vector.X() * projection_length,
            coral_block_pose.Y() + vector.Y() * projection_length
        )

        delta_pose_final = cur_pose.translation() - final_pose.translation()

        side_to_side_dist = delta_pose_final.X() * final_rotation_out.cos() + delta_pose_final.Y() * final_rotation_out.sin()
        back_and_forth_dist = delta_pose_final.X() * final_pose.rotation().cos() + delta_pose_final.Y() * final_pose.rotation().sin()
         
        
        
        if (cur_speeds.vx < 0.05 and cur_speeds.vy < 0.05) and (cur_pose.translation().distance(coral_block_pose) < 0.03) and (cur_pose.translation().distance(closest_point) < 0.01) and (self.robot.score_state.number == 2 or self.robot.score_state.number == 3):
            self.coral_blocking.appendleft(True)
        else:
            self.coral_blocking.appendleft(False)
        
        if self.at_inter_pose:
            if ((
                side_to_side_dist < 0.01
                and back_and_forth_dist < 0.03
                and (cur_speeds.vx ** 2) + (cur_speeds.vy ** 2) < 0.1
                and self.theta_controller.atSetpoint()) or all(self.coral_blocking)) and (self.robot.score_intent
                and self.robot.running_pid_lineup):
                if all(self.coral_blocking):
                    if self.robot.score_state.number == 2:
                        self.robot.score_state = RobotScoringPositions.L2_Scoring_Blocked
                    elif self.robot.score_state.number == 3:
                        self.robot.score_state = RobotScoringPositions.L3_Scoring_Blocked
                    else:
                        pass
                        #add feedback? drive out a bit or something?
                self.coral_blocking.clear()
                for i in range(self.coral_blocking_length):
                    self.coral_blocking.appendleft(False)
                self.limit_reef_acc = False
                self.acc_limit_counter = 1
                self.robot.at_scoring_position = True
                self.xy_max_acc = 4.0
                self.xy_max_vel = 4.0
                # self.x_controller.reset()
                # self.y_controller.reset()
                self.theta_controller.reset()
        else:
            if (
                # self.xy_inter_controller.atSetpoint()
                self.theta_controller.atSetpoint()
                and self.xy_inter_controller.atSetpoint()
                # and self.robot.poseEstimator.curEstPose.translation().distance(self.inter_pose.translation()) < 0.75
                and self.robot.score_intent
                and self.robot.running_pid_lineup
            ):
                self.at_inter_pose = True
                self.theta_controller.reset()
                # self.xy_inter_controller.reset()    
        self.drive(Translation2d(vx, vy), omega, True, False)

        # Update SmartDashboard values for debugging
        SmartDashboard.putNumber("t_pose x", final_pose.X())
        SmartDashboard.putNumber("t_pose y", final_pose.Y())
        SmartDashboard.putNumber("vx", vx)
        SmartDashboard.putNumber("vy", vy)
        SmartDashboard.putNumber("omega", omega)

    def go_to_pose_profiled_pid_ghost(self, final_target_pose : Pose2d, feedforward_x=0.0, feedforward_y=0.0, feedfoward_theta=0.0):
        current_pose = self.robot.poseEstimator.curEstPose

        # **Dynamically shift the pose based on current position**
        shift_factor = 0.5  # Adjust this value to control shifting effect
        xy_error = (final_target_pose.translation() - current_pose.translation()).norm()
        if xy_error < 0.1:
            shift_factor = 0.0

        face_angle = final_target_pose.rotation() - Rotation2d.fromDegrees(90)
        shift_x = shift_factor * xy_error * math.cos(face_angle.radians())
        shift_y = shift_factor * xy_error * math.sin(face_angle.radians())

        # Compute **intermediate shifted target**
        dynamic_target = Translation2d(
            final_target_pose.X() + shift_x,
            final_target_pose.Y() + shift_y
        )

        # **PID-controlled movement towards dynamic target**
        vx = self.x_controller.calculate(current_pose.X(), dynamic_target.X()) + feedforward_x
        vy = self.y_controller.calculate(current_pose.Y(), dynamic_target.Y()) + feedforward_y
        omega = self.theta_controller.calculate(
            current_pose.rotation().degrees(), final_target_pose.rotation().degrees()
        ) + feedfoward_theta

        # Check if we reached the setpoint
        if self.x_controller.atSetpoint() and self.y_controller.atSetpoint() and self.theta_controller.atSetpoint():
            if self.robot.score_intent:
                self.robot.at_scoring_position = True

        # **Drive towards dynamic pose instead of final target**
        self.drive(Translation2d(vx, vy), omega, True, False)

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

    def turn_wheels_to_x(self): #, left_source : bool):
        module_states = [
            SwerveModuleState(1, Rotation2d.fromDegrees(45)),
            SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        ]
        for idx, module in enumerate(self.robot.poseEstimator.modules):
            module.set_desired_state(module_states[idx], False)

    def reset_pid_error(self):
        self.x_controller.reset()
        self.y_controller.reset()
        self.theta_controller.reset()
        
    def get_robot_relative_speeds(self):

        module_states = (
            self.robot.poseEstimator.get_module_states()
        )  # Check this in swervemodule.py, we need to convert kraken speed to m/s
        chassis_speeds = const.SWERVE_KINEMATICS.toChassisSpeeds(module_states)  # type: ignore
        return chassis_speeds

    def shouldFlipPath(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def periodic(self):
        if self.robot.in_autonomous_mode and self.robot.running_pid_lineup:
            if self.robot.is_intaking:
                self.go_to_pose_profiled_pid(self.robot.final_lineup_pose)
            else:
                self.go_to_pose_profiled_pid(self.robot.final_lineup_pose)
        self.inter_max_vel = SmartDashboard.getNumber("Inter Max Vel", 4.0)
        self.inter_max_acc = SmartDashboard.getNumber("Inter Max Accel", 4.0)
        if not self.limit_reef_acc:
            self.xy_max_vel = SmartDashboard.getNumber("XY Max Vel", 4.0)
            self.xy_max_acc = SmartDashboard.getNumber("XY Max Acc", 4.0)
        self.xy_inter_controller.setConstraints(TrapezoidProfile.Constraints(self.inter_max_vel, self.inter_max_acc))
        self.xy_controller.setConstraints(TrapezoidProfile.Constraints(self.xy_max_vel, self.xy_max_acc))

        if not self.robot.in_autonomous_mode:
            cur_speeds = const.SWERVE_KINEMATICS.toChassisSpeeds(self.robot.poseEstimator.get_module_states())
            if (not self.robot.running_pid_lineup) and (math.sqrt((cur_speeds.vx ** 2) + (cur_speeds.vy ** 2)) < 0.02) and (cur_speeds.omega_dps < 1.0) and (abs(self.robot.oi.driver1.LEFT_JOY_X()) < 0.05
                and abs(self.robot.oi.driver1.LEFT_JOY_Y()) < 0.05
                and abs(self.robot.oi.driver1.RIGHT_JOY_X()) < 0.1
                and abs(self.robot.oi.driver1.RIGHT_JOY_Y()) < 0.1):
                    self.robot.wheels_at_x = True
            else:
                    self.robot.wheels_at_x = False


    def log(self):
        # if self.robot.running_pid_lineup:
        #     object = self.robot.poseEstimator.field_for_single_tag.getObject("lineup curve" + str(self.counter))
        #     object.setPose(self.robot.poseEstimator.curEstPose)
        SmartDashboard.putBoolean("Wheels to X", self.robot.wheels_at_x)
        self.counter += 1
        SmartDashboard.putNumber("XY Max Vel", self.xy_max_vel)
        SmartDashboard.putNumber("XY Max Acc", self.xy_max_acc)
        SmartDashboard.putBoolean("Limit Reef Acc", self.limit_reef_acc)
        SmartDashboard.putNumber("Reef Acc Counter", self.acc_limit_counter)
        SmartDashboard.putNumber("Inter Max Vel", self.inter_max_vel)
        SmartDashboard.putNumber("Inter Max Accel", self.inter_max_acc)
        SmartDashboard.putData("PID Controller Reef XY", self.xy_controller)
        SmartDashboard.putData("PID Controller for going to reef, x", self.x_controller)
        SmartDashboard.putData("PID Controller for going to reef, y", self.y_controller)
        SmartDashboard.putData(
            "PID Controller for going to reef, theta", self.theta_controller
        )
        SmartDashboard.putBoolean("At Inter Pose", self.at_inter_pose)

        SmartDashboard.putData("PID Controller (Drivetrain)", self.angle_pid)
        SmartDashboard.putBoolean("Angle at Setpoint", self.angle_pid.atSetpoint())
        SmartDashboard.putNumber("PID Controller Error", self.angle_pid.getError())
        SmartDashboard.putData("PID Controller (XY Inter)", self.xy_inter_controller)
        SmartDashboard.putBoolean("Controller Blocking", all(self.coral_blocking))