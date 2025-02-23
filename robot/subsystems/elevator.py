from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot


import time
import math
from phoenix6.hardware import TalonFX
from wpilib import SmartDashboard, Timer
from phoenix6 import configs, hardware, controls, signals
from commands2 import Subsystem
import wpilib

from wpimath.geometry import (
    Translation2d,
    Pose2d,
    Rotation2d
)
from path_gen import PathGenerator
import const
from robot_scoring_positions import RobotScoringPositions

class Elevator(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.path_generator = PathGenerator(Pose2d(), Pose2d(1,1, Rotation2d(0))) #dummy class to check for obstacles
        self.elevator_motor_1 = hardware.TalonFX(const.ELEVATOR_MOTOR_1_CAN_ID, "rio")
        self.elevator_motor_2 = hardware.TalonFX(const.ELEVATOR_MOTOR_2_CAN_ID, "rio")

        self.elevator_motor_config = configs.TalonFXConfiguration()

        ## First we will set everything to zero and then adjust k_g until the elevator is able to hold its position when we command a position
        self.elevator_motor_config.slot0.k_g = 0.45

        ## Next we will adjust k_s until the elevator just barely moves when we command a position (check both up and down)
        self.elevator_motor_config.slot0.k_s = 0.0

        ## Next we will adjust k_p until the elevator moves to the correct position and slightly overshoots/oscillates
        self.elevator_motor_config.slot0.k_p = 4.0
        ## Next we will adjust k_d until the elevator moves to the correct position without overshooting/oscillating
        self.elevator_motor_config.slot0.k_d = 0.0

        ## Adjust other stuff if we need it like k_v and k_a for feed forward
        self.elevator_motor_config.current_limits.supply_current_limit = 80
        self.elevator_motor_config.torque_current.peak_forward_torque_current = 80
        self.elevator_motor_config.torque_current.peak_reverse_torque_current = -80
        ##Ramps
        self.elevator_motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        self.elevator_motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        self.elevator_motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        self.elevator_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        self.elevator_motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        self.elevator_motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02
        self.elevator_motor_config.current_limits.supply_current_limit_enable = True
        self.elevator_motor_config.motor_output.neutral_mode = signals.NeutralModeValue(1)
        self.elevator_motor_config.current_limits.stator_current_limit = 100

        self.elevator_motor_config.motor_output.inverted = signals.InvertedValue(1)

        # We will adjust these values later to get the elevator moving faster
        self.elevator_motor_config.motion_magic.motion_magic_cruise_velocity = 200 # Recalc has us at 16 RPS, but starting slow
        self.elevator_motor_config.motion_magic.motion_magic_acceleration = 150 # Recalc has us at 100 RPS/s^2, but starting slow

        self.elevator_motor_1.configurator.apply(self.elevator_motor_config)  # type: ignore
        self.elevator_motor_2.configurator.apply(self.elevator_motor_config)  # type: ignore

        self.elevator_motor_2.set_control(controls.Follower(const.ELEVATOR_MOTOR_1_CAN_ID, False)) # Set the second motor to follow the first one but inverted

        ## Use these values to convert rotations to inches and vice versa for motion magic commands
        self.sprocket_diameter = 1.273 # in. for 16t, need to adjust if using something else
        self.gear_ratio = 6.176 # need to adjust if using something else
        self.command_height = 0.0

        ## self.request = controls.DynamicMotionMagicTorqueCurrentFOC(0.0) We can use dynamic motion magic to change the cruise velocity and acceleration on the fly, less acceration when the elevator comes down, etc.
        self.request = controls.MotionMagicVoltage(0, enable_foc=True)
        ## Somewhere here we want to set the position of the motor to the absolute encoder value with some offset for the starting position of the encoder
        # self.elevator_motor_1.set_position(0.0)
        self.elevator_pitch_roll_greater_10 = False
        self.in_proximity_to_begin_raising_elevator = False

        # self.height_encoder = wpilib.DutyCycleEncoder(0)
        # self.elevator_motor_1.set_position(self.height_encoder.get())

    def stop(self):
        self.elevator_motor_1.set_control(controls.PositionVoltage(0.0, enable_foc=True))

    def get_height(self):
        rotations = self.elevator_motor_1.get_position().value
        height = rotations / self.gear_ratio * math.pi * self.sprocket_diameter * 3  # 3 is the mechanical advantage of the elevator
        return height

    def set_elevator_height(self, height):
        if abs(self.get_height() - height) <= 0.25:
            return
        self.command_height = height
        sprocket_rotations = height / (math.pi * self.sprocket_diameter * 3) # some math for height here
        rotation = sprocket_rotations * self.gear_ratio
        self.elevator_motor_1.set_control(self.request.with_position(rotation))
        # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html

    def elevator_to_top(self):
        self.set_elevator_height(62)
    def elevator_to_bottom(self):
        self.set_elevator_height(0)

    def periodic(self):
        ## If limit switch is hit,
            ## Stop the motors and set the position to 0 or the maximum height
            ## Hopefully this prevents the elevator from breaking
        if self.robot.oi.score_intent:
            if (self.robot.poseEstimator.curEstPose - self.robot.oi.final_lineup_pose).norm() < 2 and\
                not (self.path_generator.obstacleBetween(self.robot.poseEstimator.curEstPose, self.robot.oi.final_lineup_pose)):
                #raise elevator
                self.in_proximity_to_begin_raising_elevator = True
                self.robot.end_effector.set_end_effector_position(self.robot.score_state.end_effector_position)
                self.set_elevator_height(self.robot.score_state.elevator_height)
            else:
                self.in_proximity_to_begin_raising_elevator = False
        elif self.robot.oi.manual_scoring:
            self.robot.end_effector.set_end_effector_position(self.robot.score_state.end_effector_position)
            self.set_elevator_height(self.robot.score_state.elevator_height)
        elif self.robot.mechanisms_at_default:
            self.set_elevator_height(RobotScoringPositions.elevator_intake_height)

		#bring elevator down if pitch | roll is greater than 10 degrees
        if self.robot.poseEstimator.gyro.get_pitch().value > 10 and self.robot.poseEstimator.gyro.get_roll().value > 10:
            self.elevator_pitch_roll_greater_10 = True
        else:
            self.elevator_pitch_roll_greater_10 = False
			# self.set_elevator_height(0)

    def log(self):
        SmartDashboard.putBoolean("At defaults", self.robot.mechanisms_at_default)
        SmartDashboard.putBoolean("Score intent", self.robot.oi.score_intent)
        SmartDashboard.putNumber("Current elevator height: ", self.get_height())
        SmartDashboard.putNumber("Commanded elevator height: ", self.command_height)
        SmartDashboard.putBoolean("elevator pitch roll >10", self.elevator_pitch_roll_greater_10)
        SmartDashboard.putBoolean("closer than 2 meters", self.in_proximity_to_begin_raising_elevator)
