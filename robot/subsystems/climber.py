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

class Climber(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.climber_winch_motor = hardware.TalonFX(const.CLIMBER_WINCH_MOTOR_CAN_ID, "rio")
        self.climber_intake_motor = hardware.TalonFX(const.CLIMBER_INTAKE_MOTOR_CAN_ID, "rio")

        self.climber_winch_motor_config = configs.TalonFXConfiguration()

        ## First we will set everything to zero and then adjust k_g until the elevator is able to hold its position when we command a position
        self.climber_winch_motor_config.slot0.k_g = 0.45

        ## Next we will adjust k_s until the elevator just barely moves when we command a position (check both up and down)
        self.climber_winch_motor_config.slot0.k_s = 0.0

        ## Next we will adjust k_p until the elevator moves to the correct position and slightly overshoots/oscillates
        self.climber_winch_motor_config.slot0.k_p = 7.0
        ## Next we will adjust k_d until the elevator moves to the correct position without overshooting/oscillating
        self.climber_winch_motor_config.slot0.k_d = 0.0

        ## Adjust other stuff if we need it like k_v and k_a for feed forward
        self.climber_winch_motor_config.current_limits.supply_current_limit = 80
        self.climber_winch_motor_config.torque_current.peak_forward_torque_current = 80
        self.climber_winch_motor_config.torque_current.peak_reverse_torque_current = -80
        ##Ramps
        self.climber_winch_motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        self.climber_winch_motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        self.climber_winch_motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        self.climber_winch_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        self.climber_winch_motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        self.climber_winch_motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02
        self.climber_winch_motor_config.current_limits.supply_current_limit_enable = True
        self.climber_winch_motor_config.motor_output.neutral_mode = signals.NeutralModeValue(1)
        self.climber_winch_motor_config.current_limits.stator_current_limit = 100

        self.climber_winch_motor_config.motor_output.inverted = signals.InvertedValue(1)

        # We will adjust these values later to get the elevator moving faster
        self.climber_winch_motor_config.motion_magic.motion_magic_cruise_velocity = 175 # Recalc has us at 16 RPS, but starting slow
        self.climber_winch_motor_config.motion_magic.motion_magic_acceleration = 100 # Recalc has us at 100 RPS/s^2, but starting slow

        self.climber_winch_motor.configurator.apply(self.climber_winch_motor_config)

        self.climber_intake_motor_config = configs.TalonFXConfiguration()
        self.climber_intake_motor_config.motor_output.inverted = signals.InvertedValue(1)
        self.climber_intake_motor_config.current_limits.supply_current_limit = 40
        self.climber_intake_motor_config.current_limits.supply_current_limit_enable = True
        self.climber_intake_motor_config.slot0.k_p = 2.5
        self.climber_intake_motor_config.slot0.k_i = 0.0
        self.climber_intake_motor_config.slot0.k_d = 0.0
        self.climber_intake_motor_config.slot0.k_v = 0.0

        self.climber_intake_motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        self.climber_intake_motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        self.climber_intake_motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        self.climber_intake_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        self.climber_intake_motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        self.climber_intake_motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02

        self.climber_intake_motor_config.motor_output.inverted = signals.InvertedValue(1)

        self.climber_intake_motor.configurator.apply(self.climber_intake_motor_config)

    def stop(self):
        pass

    def periodic(self):
        pass

    def log(self):
        pass