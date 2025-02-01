

from typing import TYPE_CHECKING

import math
from enum import Enum

from wpimath.geometry import Rotation2d, Rotation3d, Translation2d, Translation3d, Pose2d, Pose3d, Transform2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.units import inchesToMeters, degreesToRadians

from wpilib import DriverStation

class FieldConstants():
    """
    These are field constants and positions from the blue alliance side.
    (0,0): When standing at the blue driver stations, (0,0) is to the right and back (is the extension of Driver Station wall and processor wall)
    Y-axis: across the width of field
    X-axis: down the length
    """
    fieldLength = inchesToMeters(690.876)
    fieldWidth = inchesToMeters(317)
    startingLineX = inchesToMeters(299.438) #Measured from the inside of starting line
    algaeDiameter = inchesToMeters(16)
    shouldFlip = DriverStation.getAlliance == DriverStation.Alliance.kRed

    @staticmethod
    def flip_X_coord(x):
        return FieldConstants.fieldLength - x if FieldConstants.shouldFlip else x
    
    @staticmethod
    def flip_Y_coord(y):
        return FieldConstants.fieldWidth - y if FieldConstants.shouldFlip else y
    @staticmethod
    def flip_Translation2d (translation):
        return Translation2d(FieldConstants.flip_X_coord(translation.X()), FieldConstants.flip_Y_coord(translation.Y())) if FieldConstants.shouldFlip else translation
    @staticmethod
    def flip_Rotation2d(rotation):
        return rotation.rotateBy(Rotation2d.fromDegrees(180)) if FieldConstants.shouldFlip else rotation
    @staticmethod
    def flip_Pose2d(pose):
        return Pose2d(FieldConstants.flip_Translation2d(pose.translation()), FieldConstants.flip_Rotation2d(pose.rotation())) if FieldConstants.shouldFlip else pose

    class Processor():
        centerFace = Pose2d(inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90))
    
    class Barge():
        farCage = Translation2d(inchesToMeters(345.428), inchesToMeters(286.779)) # cage closest to the middle
        middleCage = Translation2d(inchesToMeters(345.428), inchesToMeters(242.855))
        closeCage = Translation2d(inchesToMeters(345.428), inchesToMeters(199.947)) #cage closest to outside wall

        #from floor to bottom of cage
        deepHeight = inchesToMeters(3.125)
        shallowHeight = inchesToMeters(30.125)
        
    class CoralStation():
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
        #from floor to bottom of cage
        deepHeight = inchesToMeters(3.125)
        shallowHeight = inchesToMeters(30.125)
        
    class CoralStation():
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

    class ReefHeight(Enum):
        L4 = (inchesToMeters(72), -90)
        L3 = (inchesToMeters(47.625),-35)
        L2 = (inchesToMeters(31.875),-35)
        L1 = (inchesToMeters(18),0)

        def __init__(self, height, pitch):
            self.height = height
            self.pitch = pitch
            

    class Reef():
        center = Translation2d(inchesToMeters(176.746), inchesToMeters(158.501))
        faceToZoneLine = inchesToMeters(12) # Side of the reef to the inside of the reef zone line
        centerFaces = [Pose2d(inchesToMeters(144.003), inchesToMeters(158.500), Rotation2d.fromDegrees(180)),
                    Pose2d(inchesToMeters(160.373), inchesToMeters(186.857), Rotation2d.fromDegrees(120)),
                    Pose2d(inchesToMeters(193.116), inchesToMeters(186.858), Rotation2d.fromDegrees(60)),
                    Pose2d(inchesToMeters(209.489), inchesToMeters(158.502), Rotation2d.fromDegrees(0)),
                    Pose2d(inchesToMeters(193.118), inchesToMeters(130.145), Rotation2d.fromDegrees(-60)),
                    Pose2d(inchesToMeters(160.375),inchesToMeters(130.144),Rotation2d.fromDegrees(-120))
                    ]# Starting facing the driver station in clockwise order
        branchPositions = []
        
        for face in range(6):
            #Right and left determined from standing outside of the reef looking at the face (not from looking from the inside of reef).
            fillRight = []
            fillLeft = []
            for level in [(inchesToMeters(72), -90), (inchesToMeters(47.625), -35), (inchesToMeters(31.875), -35), (inchesToMeters(18), 0)]:
                poseDirection = Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)))
                adjustX = inchesToMeters(30.738)
                adjustY = inchesToMeters(6.469)
                
                fillRight.append(level)
                fillRight.append(Pose3d(
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
                    ))
                fillLeft.append(level)
                fillLeft.append(Pose3d(
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
                    ))
        
            branchPositions.append(fillRight)
            branchPositions.append(fillLeft)

    class StagingPositions():
        '''Positions of the starting algae and coral on top of each other'''
        #standing at driver station facing away
        leftIceCream  = Translation2d(inchesToMeters(48), inchesToMeters(230.5))
        middleIceCream = Translation2d(inchesToMeters(48), inchesToMeters(158.5))
        rightIceCream = Translation2d(inchesToMeters(48), inchesToMeters(86.5))
#TEST PRINTING FIELD CONST VALUES


## A bunch of constants to test

# print(FieldConstants.Barge.farCage)
# print(FieldConstants.ReefHeight.L4.height)
# print(FieldConstants.Reef.centerFaces[0])
# print(FieldConstants.Reef.branchPositions)
# print(range(len(FieldConstants.Reef.branchPositions)))
# print(len(FieldConstants.Reef.branchPositions))
# print(FieldConstants.Reef.branchPositions)
#print(FieldConstants.Reef.branchPositions)

for idx in range(len(FieldConstants.Reef.branchPositions)):
    for level in range(4):
        reefHeightLevels = {
             FieldConstants.ReefHeight.L4 : "4",
             FieldConstants.ReefHeight.L3 : "3",
             FieldConstants.ReefHeight.L2 : "2",
             FieldConstants.ReefHeight.L1 : "1"
        }
        for reef_height, lvl in reefHeightLevels.items():
             if math.isclose(FieldConstants.Reef.branchPositions[idx][level*2][0], reef_height.height, abs_tol=1e-6):
                branch_level = lvl
             
        print("Face", ((idx // 2) + 1),
            ", right-branch" if idx % 2 else ", left-branch",
            ", L" + branch_level,
            "Pitch:", FieldConstants.Reef.branchPositions[idx][level * 2][1],
            "\n Pose3d: \n", FieldConstants.Reef.branchPositions[idx][(level*2) + 1],
            end="\n\n"
            )