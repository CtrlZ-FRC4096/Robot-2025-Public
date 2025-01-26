"""
These are field constants and positions from the blue alliance side.
(0,0): When standing at the blue driver stations, (0,0) is to the right and back (is the extension of Driver Station wall and processor wall)
Y-axis: across the width of field
X-axis: down the length
"""

import math
from enum import Enum

from wpimath.geometry import Rotation2d, Rotation3d, Translation2d, Translation3d, Pose2d, Pose3d, Transform2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.units import inchesToMeters, degreesToRadians

from phoenix6 import signals
from wpilib import DriverStation

'''alliance flip util'''
def flip_X_coord(self, x):
    return FieldConstants.fieldLength - x #if self.shouldFlip else x
def flip_Y_coord(self, y):
    return FieldConstants.fieldWidth - y #if self.shouldFlip else y
def flip_Translation2d (self, translation):
    return Translation2d(self.flip_X_coord(translation.getX()), self.flip_Y_coord(translation.getY())) #if self.shouldFlip else translation
def flip_Rotation2d(self, rotation):
    return rotation.rotateBy(Rotation2d.kPi) #if self.shouldFlip else rotation
def flip_Pose2d(self, pose):
    return Pose2d(self.flip_Translation2d(pose.getTranslation()), self.flip_Rotation2d(pose.getRotation())) #if self.shouldFlip else pose
#def flip_VehicleState(self, state):

class FieldConstants():
    fieldLength = inchesToMeters(690.876)
    fieldWidth = inchesToMeters(317)
    startingLineX = inchesToMeters(299.438) #Measured from the inside of starting line
    #shouldFlip = DriverStation.getAlliance.get() == DriverStation.Alliance.kRed

    @staticmethod
    def flip_X_coord(x):
        return FieldConstants.fieldLength - x #if self.shouldFlip else x
    @staticmethod
    def flip_Y_coord(y):
        return FieldConstants.fieldWidth - y #if self.shouldFlip else y
    @staticmethod
    def flip_Translation2d (translation):
        return Translation2d(flip_X_coord(translation.getX()), flip_Y_coord(translation.getY())) #if self.shouldFlip else translation
    @staticmethod
    def flip_Rotation2d(rotation):
        return rotation.rotateBy(Rotation2d.kPi) #if self.shouldFlip else rotation
    @staticmethod
    def flip_Pose2d(pose):
        return Pose2d(flip_Translation2d(pose.getTranslation()), flip_Rotation2d(pose.getRotation())) #if self.shouldFlip else pose
    
    @staticmethod
    def flip_Pose2dOnList(pose_list):
        for idx in range(len(pose_list)):
            pose_list[idx] = flip_Pose2d(pose_list[idx])
        return pose_list
    #def flip_VehicleState(self, state):

    class Blue():
        
        class Processor():
            centerFace =   Pose2d(inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90))
        
        class Barge():
            farCage = Translation2d(inchesToMeters(345.428), inchesToMeters(286.779)) # cage closest to the middle
            middleCage = Translation2d(inchesToMeters(345.428), inchesToMeters(242.855))
            closeCage = Translation2d() #cage closest to outside wall

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
            #reefHeights = ReefHeight
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
                fillRight = {}
                fillLeft = {}
                #[(inchesToMeters(72), -90), (inchesToMeters(47.625), -35), (inchesToMeters(31.875), -35), (inchesToMeters(18), 0)]
                for level in [(inchesToMeters(72), -90), (inchesToMeters(47.625), -35), (inchesToMeters(31.875), -35), (inchesToMeters(18), 0)]:
                    poseDirection = Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)))
                    adjustX = inchesToMeters(30.738)
                    adjustY = inchesToMeters(6.469)
                    fillRight[level] = Pose3d(
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
                    fillLeft[level] = Pose3d(
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
                    branchPositions.insert((face * 2) + 1, fillRight)
                    branchPositions.insert((face * 2) + 2, fillLeft)

        
        class StagingPositions():
            '''Positions of the starting algae and coral on top of each other'''
            #standing at driver station facing away
            leftIceCream  = Translation2d(inchesToMeters(48), inchesToMeters(230.5))
            middleIceCream = Translation2d(inchesToMeters(48), inchesToMeters(158.5))
            rightIceCream = Translation2d(inchesToMeters(48), inchesToMeters(86.5))
    class Red():
        class Processor():
            centerFace =   flip_Pose2d(Pose2d(inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90)))
        
        class Barge():
            farCage = flip_Translation2d(Translation2d(inchesToMeters(345.428), inchesToMeters(286.779))) # cage closest to the middle
            middleCage = flip_Translation2d(Translation2d(inchesToMeters(345.428), inchesToMeters(242.855)))
            closeCage = flip_Translation2d(Translation2d(inchesToMeters(345.428), inchesToMeters(199.947))) #cage closest to outside wall

            #from floor to bottom of cage
            deepHeight = inchesToMeters(3.125)
            shallowHeight = inchesToMeters(30.125)
            
        class CoralStation():
            leftCenterFace = flip_Pose2d(Pose2d(
                inchesToMeters(33.526), 
                inchesToMeters(291.176),
                Rotation2d.fromDegrees(90 - 144.011)
            ))
            rightCenterFace = flip_Pose2d(Pose2d(
                inchesToMeters(33.526), 
                inchesToMeters(25.824),
                Rotation2d.fromDegrees(144.011 - 90)
            ))

        class ReefHeight(Enum):
            L4 = (inchesToMeters(72), -90)
            L3 = (inchesToMeters(47.625),-35)
            L2 = (inchesToMeters(31.875),-35)
            L1 = (inchesToMeters(18),0)

            def __init__(self, height, pitch):
                self.height = height
                self.pitch = pitch
                

        class Reef():
            center = flip_Translation2d(Translation2d(inchesToMeters(176.746), inchesToMeters(158.501)))
            #reefHeights = ReefHeight
            faceToZoneLine = inchesToMeters(12) # Side of the reef to the inside of the reef zone line
            centerFaces = flip_Pose2dOnList([Pose2d(inchesToMeters(144.003), inchesToMeters(158.500), Rotation2d.fromDegrees(180)),
                        Pose2d(inchesToMeters(160.373), inchesToMeters(186.857), Rotation2d.fromDegrees(120)),
                        Pose2d(inchesToMeters(193.116), inchesToMeters(186.858), Rotation2d.fromDegrees(60)),
                        Pose2d(inchesToMeters(209.489), inchesToMeters(158.502), Rotation2d.fromDegrees(0)),
                        Pose2d(inchesToMeters(193.118), inchesToMeters(130.145), Rotation2d.fromDegrees(-60)),
                        Pose2d(inchesToMeters(160.375),inchesToMeters(130.144),Rotation2d.fromDegrees(-120))
                        ])# Starting facing the driver station in clockwise order
            branchPositions = []
            
            for face in range(6):
                fillRight = {}
                fillLeft = {}
                #[(inchesToMeters(72), -90), (inchesToMeters(47.625), -35), (inchesToMeters(31.875), -35), (inchesToMeters(18), 0)]
                for level in [(inchesToMeters(72), -90), (inchesToMeters(47.625), -35), (inchesToMeters(31.875), -35), (inchesToMeters(18), 0)]:
                    poseDirection = Pose2d(center, Rotation2d.fromDegrees(-60 * face if face <= 2 else 360 - 60 * face))
                    adjustX = inchesToMeters(30.738)
                    adjustY = inchesToMeters(6.469)
                    fillRight[level] = Pose3d(
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
                    fillLeft[level] = Pose3d(
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
                    branchPositions.insert((face * 2) + 1, fillRight)
                    branchPositions.insert((face * 2) + 2, fillLeft)

        
        class StagingPositions():
            '''Positions of the starting algae and coral on top of each other'''
            #standing at driver station facing away
            leftIceCream  = Translation2d(inchesToMeters(48), inchesToMeters(230.5))
            middleIceCream = Translation2d(inchesToMeters(48), inchesToMeters(158.5))
            rightIceCream = Translation2d(inchesToMeters(48), inchesToMeters(86.5))

