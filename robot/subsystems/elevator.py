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

import const

class Elevator(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.elevator_motor_1 = hardware.TalonFX(const.ELEVATOR_MOTOR_1_CAN_ID, "rio")
        self.elevator_motor_2 = hardware.TalonFX(const.ELEVATOR_MOTOR_2_CAN_ID, "rio")

        self.elevator_motor_config = configs.TalonFXConfiguration()
        self.elevator_motor_config.slot0.k_p = 2.2  # 2.2
        self.elevator_motor_config.slot0.k_s = 3.5
        self.elevator_motor_config.slot0.k_v = 0.24  # 0.24
        ## Feed Forward
        # self.elevator_motor_config.slot0.k_v = const.SWERVE_DRIVE_KV
        # self.elevator_motor_config.slot0.k_a = const.SWERVE_DRIVE_KA
        self.elevator_motor_config.current_limits.supply_current_limit = (
            80  # I am not sure if this is correct
        )

        self.elevator_motor_config.torque_current.peak_forward_torque_current = (
            80  # Up this to 80 for more zip
        )
        self.elevator_motor_config.torque_current.peak_reverse_torque_current = -80
        ##Ramps
        self.elevator_motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = (
            0.02
        )
        self.elevator_motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        self.elevator_motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = (
            0.02
        )
        self.elevator_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = (
            0.02
        )
        self.elevator_motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = (
            0.02
        )
        self.elevator_motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02

        self.elevator_motor_config.current_limits.supply_current_limit_enable = True
        self.elevator_motor_config.motor_output.neutral_mode = signals.NeutralModeValue(
            1
        )
        self.elevator_motor_config.current_limits.stator_current_limit = 100

        self.elevator_motor_config.motion_magic.motion_magic_cruise_velocity = 100
        self.elevator_motor_config.motion_magic.motion_magic_acceleration = 1000

        self.elevator_motor_1.configurator.apply(self.elevator_motor_config)  # type: ignore

        self.elevator_motor_config.motor_output.inverted = signals.InvertedValue(1)
        self.elevator_motor_2.configurator.apply(self.elevator_motor_config)  # type: ignore

        self.elevator_encoder = wpilib.DutyCycleEncoder(0)

        self.max_height = 100 # need to adjust later
        self.min_height = 0 # need to adjust later


    def stop(self):
        self.elevator_motor_1.set_control(controls.VelocityTorqueCurrentFOC(0.0))
        self.elevator_motor_2.set_control(controls.VelocityTorqueCurrentFOC(0.0))

    def get_height(self):
        return (self.elevator_motor_1.get_position().value + self.elevator_motor_2.get_position().value) / 2

    def set_elevator_height(self, height):
        self.elevator_motor_1.set_control(controls.MotionMagicTorqueCurrentFOC(height))
        self.elevator_motor_2.set_control(controls.MotionMagicTorqueCurrentFOC(height))

    def periodic(self):
        pass

    def log(self):
        pass