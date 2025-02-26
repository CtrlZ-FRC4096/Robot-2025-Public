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


class FunnelIntake(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.intake_motor = hardware.TalonFX(const.FUNNEL_INTAKE_MOTOR_CAN_ID, "rio")

        funnel_intake_config = configs.TalonFXConfiguration()  # apply config file
        funnel_intake_config.motor_output.inverted = signals.InvertedValue(1)
        funnel_intake_config.current_limits.supply_current_limit = 40
        funnel_intake_config.current_limits.supply_current_limit_enable = True
        funnel_intake_config.slot0.k_p = const.SWERVE_DRIVE_KP
        funnel_intake_config.slot0.k_i = const.SWERVE_DRIVE_KI
        funnel_intake_config.slot0.k_d = const.SWERVE_DRIVE_KD
        funnel_intake_config.slot0.k_v = const.SWERVE_DRIVE_KF

        funnel_intake_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        funnel_intake_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        funnel_intake_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        funnel_intake_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        funnel_intake_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        funnel_intake_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02

        self.intake_motor.configurator.apply(funnel_intake_config)  # type: ignore

        self.canrange_funnel = CANrange(const.FUNNEL_CANRANGE_ID, "rio")

        self.piece_passing_through_now = False
        self.piece_passing_through_previous_tick = False

        self.canrange_funnel_config = configs.CANrangeConfiguration()
        self.canrange_funnel_prox_config = ProximityParamsConfigs()
        self.canrange_funnel_prox_config.proximity_threshold = 0.09
        # self.canrange_funnel_prox_config.proximity_hysteresis = 0.0508  # +- 2 inches
        self.canrange_funnel_config.with_proximity_params(self.canrange_funnel_prox_config)

        self.canrange_funnel.configurator.apply(self.canrange_funnel_config)

        self.commanded_speed = 0.0
        self.is_intaking = False
        deque_length = 2
        self.piece_detected = deque(maxlen=deque_length)
        for i in range(deque_length):
            self.piece_detected.append(False)

    def stop(self):
        self.intake_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))

    def intake(self, speed=150):
        self.commanded_speed = speed
        if abs(self.intake_motor.get_velocity().value - self.commanded_speed) <= 0.25:
            return
        self.intake_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def periodic(self):
        if self.is_intaking:
            self.intake(100)
            # self.piece_detected.appendleft(self.canrange_funnel.get_is_detected().value) # automatically pops oldest when over 3
            # self.piece_passing_through_previous_tick = self.piece_passing_through_now
            # self.piece_passing_through_now = all(self.piece_detected)
            # if not self.piece_passing_through_now and self.piece_passing_through_previous_tick:
            #     self.is_intaking = False
            #     self.stop()
        elif self.robot.mechanisms_at_default:
            self.piece_passing_through = False
            self.stop()

    def log(self):
        SmartDashboard.putBoolean("funnel is intaking", self.is_intaking)
        SmartDashboard.putBoolean("piece passing through funnel", self.piece_passing_through_now)
        SmartDashboard.putBoolean("piece in funnel now", all(self.piece_detected))
        SmartDashboard.putNumber("funnel intake speed", self.intake_motor.get_velocity().value)
        SmartDashboard.putBoolean("funnel canrange detecting piece", self.canrange_funnel.get_is_detected().value)
        SmartDashboard.putNumber("funnel canrange distance", self.canrange_funnel.get_distance().value)
        SmartDashboard.putNumber("funnel commanded intake speed", self.commanded_speed)
