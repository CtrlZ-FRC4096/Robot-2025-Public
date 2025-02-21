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
from phoenix6.configs import CANcoderConfigurator
from phoenix6.configs.config_groups import ProximityParamsConfigs

class EndEffector(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.end_effector_motor = hardware.TalonFX(const.END_EFFECTOR_MOTOR_CAN_ID, "rio")

        end_effector_config = configs.TalonFXConfiguration()  # apply config file
        end_effector_config.motor_output.inverted = signals.InvertedValue(0) #FIND IF MOTOR SHOULD BE INVERTED
        end_effector_config.current_limits.supply_current_limit = 40
        end_effector_config.current_limits.supply_current_threshold = 0
        end_effector_config.current_limits.supply_time_threshold = 0
        end_effector_config.current_limits.supply_current_limit_enable = True
        end_effector_config.slot0.k_p = const.SWERVE_DRIVE_KP
        end_effector_config.slot0.k_i = const.SWERVE_DRIVE_KI
        end_effector_config.slot0.k_d = const.SWERVE_DRIVE_KD
        end_effector_config.slot0.k_v = const.SWERVE_DRIVE_KF

        end_effector_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        end_effector_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        end_effector_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        end_effector_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        end_effector_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        end_effector_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02

        self.end_effector_motor.configurator.apply(end_effector_config)
        self.isRunning = False

        self.canrange_end_effector = CANrange(const.END_EFFECTOR_CANRANGE_ID, "rio")

        self.canrange_end_effector_config = configs.CANrangeConfiguration()
        self.canrange_end_effector_prox_config = ProximityParamsConfigs()
        # CHANGE PROXIMITY STUFF
        self.canrange_end_effector_prox_config.proximity_threshold = 0.3048  # 1 foot
        self.canrange_end_effector_prox_config.proximity_hysteresis = 0.0508  # +- 2 inches
        self.canrange_end_effector_config.with_proximity_params(self.canrange_end_effector_prox_config)

        self.canrange_end_effector.configurator.apply(self.canrange_end_effector_config)

    def stop(self):
        self.end_effector_motor.set_control(controls.VelocityTorqueCurrentFOC(0))

    def score(self, speed=83): #CHANGE DEFAULT SPEED
        self.end_effector_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def periodic(self):
        pass

    def log(self):
        pass


