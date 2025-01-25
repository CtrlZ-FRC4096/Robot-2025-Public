"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

"""
Prepend these to any port IDs.
DIO = Digital I/O
AIN = Analog Input
PWM = Pulse Width Modulation
CAN = Controller Area Network
PCM = Pneumatic Control Module
PDP = Power Distribution Panel
"""

import math

from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics

from phoenix6 import signals

def inchesToMeters(inch):
    return inch/39.37

class FieldConstants():
    def __init__(self):
        self.fieldLength = inchesToMeters(690.876)
        self.fieldWidth = inchesToMeters(317)
        self.startingLineX = inchesToMeters(299.438) #Measured from the inside of starting line
    
    @property
    def Processor(self) -> None:
        self.centerFace = Pose2d(inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90))
    
    @property
    def Barge(self) -> None:
        self.farCage = Translation2d(inchesToMeters(345.428), inchesToMeters(286.779)) # cage closest to the middle
        self.middleCage = Translation2d(inchesToMeters(345.428), inchesToMeters(242.855))
        self.closeCage = Translation2d() #cage closest to outside wall
    @property
    def CoralStation(self) -> None:
        
        
        
        

