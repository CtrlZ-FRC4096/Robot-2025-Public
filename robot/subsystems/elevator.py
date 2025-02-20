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

class Elevator(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.path_generator = PathGenerator(Pose2d(), Pose2d(1,1, Rotation2d(0))) #dummy class to check for obstacles
        self.elevator_motor_1 = hardware.TalonFX(const.ELEVATOR_MOTOR_1_CAN_ID, "rio")
        self.elevator_motor_2 = hardware.TalonFX(const.ELEVATOR_MOTOR_2_CAN_ID, "rio")

        self.elevator_motor_config = configs.TalonFXConfiguration()
        
        ## First we will set everything to zero and then adjust k_g until the elevator is able to hold its position when we command a position
        self.elevator_motor_config.slot0.k_g = 0.0
        
        ## Next we will adjust k_s until the elevator just barely moves when we command a position (check both up and down)
        self.elevator_motor_config.slot0.k_s = 0.0
        
        ## Next we will adjust k_p until the elevator moves to the correct position and slightly overshoots/oscillates
        self.elevator_motor_config.slot0.k_p = 0.0
        ## Next we will adjust k_d until the elevator moves to the correct position without overshooting/oscillating
        self.elevator_motor_config.slot0.k_d = 0.0
        
        ## Adjust other stuff if we need it like k_v and k_a for feed forward
        
                
        
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
        
        # We will adjust these values later to get the elevator moving faster
        self.elevator_motor_config.motion_magic.motion_magic_cruise_velocity = 1 # Recalc has us at 16 RPS, but starting slow
        self.elevator_motor_config.motion_magic.motion_magic_acceleration = 10 # Recalc has us at 100 RPS/s^2, but starting slow

        self.elevator_motor_1.configurator.apply(self.elevator_motor_config)  # type: ignore

        self.elevator_motor_2.configurator.apply(self.elevator_motor_config)  # type: ignore
        
        self.elevator_motor_2.set_control(controls.Follower(const.ELEVATOR_MOTOR_1_CAN_ID, True)) # Set the second motor to follow the first one but inverted

        self.max_height = 100 # need to adjust later
        self.min_height = 0 # need to adjust later
        
        ## Use these values to convert rotations to inches and vice versa for motion magic commands
        sprocket_diameter = 1.273 # in. for 16t, need to adjust if using something else
        gear_ration = 6.176 # need to adjust if using something else
        
        ## self.request = controls.DynamicMotionMagicTorqueCurrentFOC(0.0) We can use dynamic motion magic to change the cruise velocity and acceleration on the fly, less acceration when the elevator comes down, etc.
        self.request = controls.MotionMagicTorqueCurrentFOC(0.0)
        
        ## Somewhere here we want to set the position of the motor to the absolute encoder value with some offset for the starting position of the encoder
        # self.elevator_motor_1.set_position(0.0)

    def stop(self):
        self.elevator_motor_1.set_control(controls.VelocityTorqueCurrentFOC(0.0))

    def get_height(self):
        return self.elevator_motor_1.get_position().value

    def set_elevator_height(self, height):
        rotation = height ##some math for height here
        self.elevator_motor_1.set_control(self.request.with_position(rotation))
        # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html

    def periodic(self):
        ## If limit switch is hit,
            ## Stop the motors and set the position to 0 or the maximum height
            ## Hopefully this prevents the elevator from breaking
        
        if self.robot.poseEstimator.score_intent:
            if (self.robot.poseEstimator.curEstPose - self.robot.oi.final_lineup_pose).norm() < 2 and\
                not (self.path_generator.obstacleBetween(self.robot.poseEstimator.curEstPose, self.robot.oi.final_lineup_pose)):
                #raise elevator
                pass
        
        pass

    def log(self):
        pass