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

from wpimath.geometry import Rotation2d, Rotation3d, Translation2d, Translation3d, Pose2d, Pose3d, Transform2d
from wpimath.kinematics import SwerveDrive4Kinematics

from phoenix6 import signals

def inchesToMeters(inch):
    return inch/39.37
def degreesToRadians(deg):
    return deg * math.pi/180

class FieldConstants():
    fieldLength = inchesToMeters(690.876)
    fieldWidth = inchesToMeters(317)
    startingLineX = inchesToMeters(299.438) #Measured from the inside of starting line

    class Processor:
        centerFace = Pose2d(inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90))
    
    class Barge:
        farCage = Translation2d(inchesToMeters(345.428), inchesToMeters(286.779)) # cage closest to the middle
        middleCage = Translation2d(inchesToMeters(345.428), inchesToMeters(242.855))
        closeCage = Translation2d() #cage closest to outside wall
        
    class CoralStation:
        leftCenterFace = Pose2d(
            inchesToMeters(33.526), 
            inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011)
        )
        rightCenterFace = Pose2d(
            inchesToMeters(33.526), 
            inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90)
        )

    class Reef:
        center = Translation2d(inchesToMeters(176.746), inchesToMeters(158.501))
        faceToZoneLine = inchesToMeters(12) # Side of the reef to the inside of the reef zone line
        centerFaces = [0, 0, 0, 0, 0, 0] # Starting facing the driver station in clockwise order
        centerFaces[0] = Pose2d(
              inchesToMeters(144.003),
              inchesToMeters(158.500),
              Rotation2d.fromDegrees(180)
            )
        centerFaces[1] = Pose2d(
                inchesToMeters(160.373),
                inchesToMeters(186.857),
                Rotation2d.fromDegrees(120)
            )
        centerFaces[2] = Pose2d(
                inchesToMeters(193.116),
                inchesToMeters(186.858),
                Rotation2d.fromDegrees(60)
            )
        centerFaces[3] = Pose2d(
                inchesToMeters(209.489),
                inchesToMeters(158.502),
                Rotation2d.fromDegrees(0)
            )
        centerFaces[4] = Pose2d(
                inchesToMeters(193.118),
                inchesToMeters(130.145),
                Rotation2d.fromDegrees(-60)
            )
        centerFaces[5] = Pose2d(
                inchesToMeters(160.375),
                inchesToMeters(130.144),
                Rotation2d.fromDegrees(-120)
            )
        branchPositions = []
        for face in range(6):
            fillRight = []
            fillLeft = []
            for level in [(inchesToMeters(72), -90), (inchesToMeters(47.625), -35), (inchesToMeters(31.875), -35), (inchesToMeters(18), 0)]:
                poseDirection = Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)))
                adjustX = inchesToMeters(30.38)
                adjustY = inchesToMeters(6.469)
                fillRight.append(
                    level,
                    Pose3d(
                        Translation3d(
                            poseDirection.transformBy(Transform2d(adjustX, adjustY, Rotation2d())).X(),
                            poseDirection.transformBy(Transform2d(adjustX, adjustY, Rotation2d())).Y(),
                            level[0]
                        ),
                        Rotation3d(
                            0,
                            degreesToRadians(level[1]),
                            poseDirection.rotation().radians()
                        )
                    )
                )
                fillLeft.append(
                    level,
                    Pose3d(
                        Translation3d(
                            poseDirection.transformBy(Transform2d(adjustX, -adjustY, Rotation2d())).X(),
                            poseDirection.transformBy(Transform2d(adjustX, -adjustY, Rotation2d())).Y(),
                            level[0]
                        ),
                        Rotation3d(
                            0,
                            degreesToRadians(level[1]),
                            poseDirection.rotation().radians()
                        )
                    )
                )
                branchPositions.append((face * 2) + 1, fillRight)
                branchPositions.append((face * 2) + 2, fillLeft)

    class StagingPositions:
        leftIceCream  = Pose2d(Translation2d(inchesToMeters(48), inchesToMeters(230.5), Rotation2d()))
        middleIceCream = Pose2d(Translation2d(inchesToMeters(48), inchesToMeters(158.5), Rotation2d()))
        rightIceCream = Pose2d(Translation2d(inchesToMeters(48), inchesToMeters(86.5), Rotation2d()))

    class ReefHeight: # this is not used
        class L4:
            height = inchesToMeters(72)
            pitch = -90
        class L3:
            height = inchesToMeters(47.625)
            pitch = -35
        class L2:
            height = inchesToMeters(31.875)
            pitch = -35
        class L1:
            height = inchesToMeters(18)
            pitch = 0