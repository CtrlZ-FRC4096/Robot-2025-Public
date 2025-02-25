from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot


import time
import math
from phoenix6.hardware import TalonFX
from wpilib import SmartDashboard, Timer
from phoenix6 import configs, hardware, controls, signals
from commands2 import Subsystem

import const

from collections import deque

from phoenix6.hardware import CANrange
from phoenix6.configs import CANcoderConfigurator
from phoenix6.configs.config_groups import ProximityParamsConfigs
from robot_scoring_positions import RobotScoringPositions

class EndEffector(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.end_effector_motor = hardware.TalonFX(const.END_EFFECTOR_MOTOR_CAN_ID, "rio")

        self.end_effector_config = configs.TalonFXConfiguration()  # apply config file
        self.end_effector_config.motor_output.inverted = signals.InvertedValue(0)
        self.end_effector_config.current_limits.supply_current_limit = 40
        self.end_effector_config.current_limits.supply_current_limit_enable = True

		## First we will set everything to zero and then adjust k_g until the elevator is able to hold its position when we command a position
        self.end_effector_config.slot0.k_g = 0.45

        ## Next we will adjust k_s until the elevator just barely moves when we command a position (check both up and down)
        self.end_effector_config.slot0.k_s = 0.5

        ## Next we will adjust k_p until the elevator moves to the correct position and slightly overshoots/oscillates
        self.end_effector_config.slot0.k_p = 4.0
        ## Next we will adjust k_d until the elevator moves to the correct position without overshooting/oscillating
        self.end_effector_config.slot0.k_d = 0.0

        ## Adjust other stuff if we need it like k_v and k_a for feed forward
        self.end_effector_config.current_limits.supply_current_limit = 80
        self.end_effector_config.torque_current.peak_forward_torque_current = 80
        self.end_effector_config.torque_current.peak_reverse_torque_current = -80
        ##Ramps
        self.end_effector_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        self.end_effector_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        self.end_effector_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        self.end_effector_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        self.end_effector_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        self.end_effector_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02
        self.end_effector_config.current_limits.supply_current_limit_enable = True
        self.end_effector_config.motor_output.neutral_mode = signals.NeutralModeValue(1)
        self.end_effector_config.current_limits.stator_current_limit = 100

        self.end_effector_config.motion_magic.motion_magic_cruise_velocity = 100 # Recalc has us at 16 RPS, but starting slow
        self.end_effector_config.motion_magic.motion_magic_acceleration = 100 # Recalc has us at 100 RPS/s^2, but starting slow

        self.end_effector_motor.configurator.apply(self.end_effector_config)
        self.isRunning = False

        self.canrange_end_effector = CANrange(const.END_EFFECTOR_CANRANGE_ID, "rio")

        self.canrange_end_effector_config = configs.CANrangeConfiguration()
        self.canrange_end_effector_prox_config = ProximityParamsConfigs()
        # CHANGE PROXIMITY STUFF
        self.canrange_end_effector_prox_config.proximity_threshold = 0.3048  # 1 foot
        self.canrange_end_effector_config.with_proximity_params(self.canrange_end_effector_prox_config)

        self.canrange_end_effector.configurator.apply(self.canrange_end_effector_config)

        self.sprocket_diameter = 1.790 # in.
        self.gear_ratio = 4.0 # need to adjust if using something else
        self.command_position = 0.0

        ## self.request = controls.DynamicMotionMagicTorqueCurrentFOC(0.0) We can use dynamic motion magic to change the cruise velocity and acceleration on the fly, less acceration when the elevator comes down, etc.
        self.request = controls.MotionMagicVoltage(0, enable_foc=True)

        self.outtake_motor = hardware.TalonFX(const.END_EFFECTOR_OUTTAKE_MOTOR_CAN_ID, "rio")
        self.outtake_motor_config = configs.TalonFXConfiguration()  # apply config file
        self.outtake_motor_config.motor_output.inverted = signals.InvertedValue(0)
        self.outtake_motor_config.current_limits.supply_current_limit = 40
        self.outtake_motor_config.current_limits.supply_current_limit_enable = True
        self.outtake_motor_config.slot0.k_p = const.SWERVE_DRIVE_KP
        self.outtake_motor_config.slot0.k_i = const.SWERVE_DRIVE_KI
        self.outtake_motor_config.slot0.k_d = const.SWERVE_DRIVE_KD
        self.outtake_motor_config.slot0.k_v = const.SWERVE_DRIVE_KF

        self.outtake_motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        self.outtake_motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        self.outtake_motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        self.outtake_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        self.outtake_motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        self.outtake_motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02
        self.outtake_motor.configurator.apply(self.outtake_motor_config)

        self.set_end_effector_position(RobotScoringPositions.end_effector_travel_position)

        self.is_intaking = False

        self.commanded_outtake_motor_speed = 0.0

        self.piece_passing_through_now = False
        self.piece_passing_through_previous_tick = False

        self.max_extension = 9.0

        self.piece_detected = deque(maxlen=2)

    def stop(self):
        # self.end_effector_motor.set_control(controls.PositionVoltage(0.0, enable_foc=True))
        # self.outtake_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))
        pass

    def set_end_effector_position(self, position):
        if (abs(self.get_position() - position) <= 0.25):
            return
        self.command_position = position
        sprocket_rotations = position / (math.pi * self.sprocket_diameter)
        rotation = sprocket_rotations * self.gear_ratio
        self.end_effector_motor.set_control(self.request.with_position(rotation))

    def get_position(self):
        rotations = self.end_effector_motor.get_position().value
        height = rotations / self.gear_ratio * math.pi * self.sprocket_diameter
        return height

    def set_outtake_motor_speed(self, speed):
        self.commanded_outtake_motor_speed = speed
        # if abs(self.outtake_motor.get_velocity().value - self.commanded_outtake_motor_speed) <= 0.25:
        #     return
        self.outtake_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def periodic(self):
        if self.robot.score_piece or self.robot.at_scoring_position: # manual vs automated
            self.set_outtake_motor_speed(self.robot.score_state.end_effector_outtake_speed)
        elif self.is_intaking:
            self.set_end_effector_position(RobotScoringPositions.end_effector_intake_position)
            self.set_outtake_motor_speed(47.0) # default outtake speed
            self.piece_detected.append(self.canrange_end_effector.get_is_detected()) # automatically pops oldest when over 3
            self.piece_passing_through_previous_tick = self.piece_passing_through_now
            self.piece_passing_through_now = all(self.piece_detected)
            if not self.piece_passing_through_now and self.piece_passing_through_previous_tick:
                self.robot.mechanisms_at_default = True
                self.is_intaking = False
                self.robot.has_coral = True
        elif self.robot.mechanisms_at_default:
            self.stop()
            self.set_end_effector_position(RobotScoringPositions.end_effector_travel_position)


    def log(self):
        SmartDashboard.putNumber("end effector position (in)", self.get_position())
        SmartDashboard.putNumber("end effector command position (in)", self.command_position)
        SmartDashboard.putBoolean("end effector is intaking", self.is_intaking)
        SmartDashboard.putNumber("end effector outtake speed", self.outtake_motor.get_velocity().value)
        SmartDashboard.putNumber("end effector commanded outtake speed", self.commanded_outtake_motor_speed)
        SmartDashboard.putBoolean("robot is at scoring position", self.robot.at_scoring_position)
        SmartDashboard.putBoolean("end effector canrange detecting piece", self.canrange_end_effector.get_is_detected().value)
        SmartDashboard.putNumber("end effector canrange distance", self.canrange_end_effector.get_distance().value)

