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

from phoenix6.hardware import CANrange


class FunnelIntake(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.intake_motor = hardware.TalonFX(const.INTAKE_MOTOR_CAN_ID, "rio")

        funnel_intake_config = configs.TalonFXConfiguration()  # apply config file
        funnel_intake_config.motor_output.inverted = signals.InvertedValue(0)
        funnel_intake_config.current_limits.supply_current_limit = 40
        funnel_intake_config.current_limits.supply_current_threshold = 0
        funnel_intake_config.current_limits.supply_time_threshold = 0
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

        self.is_running = False
        self.canrange_1 = CANrange(const.CANRANGE_1_CAN_ID, "carnivore")

    def stop(self):
        self.intake_motor.set_control(controls.VelocityTorqueCurrentFOC(0))

    def intake(self, speed=83):
        self.intake_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))
    
    def periodic(self):
        if self.is_running:
            if self.canrange_1.get_distance() < 0.1:
                self.stop()
                self.is_running = False
            self.intake()

    def log(self):
        pass
