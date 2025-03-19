from typing import TYPE_CHECKING

import math
from enum import Enum

from wpimath.geometry import (
    Rotation2d,
    Rotation3d,
    Translation2d,
    Translation3d,
    Pose2d,
    Pose3d,
    Transform2d,
)
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.units import inchesToMeters, degreesToRadians

from wpilib import DriverStation
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
import json


class FieldConstants:
    """
    These are field constants and positions from the blue alliance side.
    (0,0): When standing at the blue driver stations, (0,0) is to the right and back (is the extension of Driver Station wall and processor wall)
    Y-axis: across the width of field
    X-axis: down the length
    """

    fieldLength = inchesToMeters(690.876)
    fieldWidth = inchesToMeters(317)
    startingLineX = inchesToMeters(299.438)  # Measured from the inside of starting line
    algaeDiameter = inchesToMeters(16)
    shouldFlip = DriverStation.getAlliance() == DriverStation.Alliance.kRed
    reef_tags = {6, 7, 8, 9, 10, 11} if shouldFlip else {17, 18, 19, 20, 21, 22}
    face_to_tag = (
        {1: 7, 2: 6, 3: 11, 4: 10, 5: 9, 6: 8}
        if shouldFlip
        else {1: 18, 2: 19, 3: 20, 4: 21, 5: 22, 6: 17}
    )
    tag_to_face = (
        {7: 1, 6: 2, 11: 3, 10: 4, 9: 5, 8: 6}
        if shouldFlip
        else {18: 1, 19: 2, 20: 3, 21: 4, 22: 5, 17: 6}
    )

    @staticmethod
    def flip_X_coord(x):
        return FieldConstants.fieldLength - x if FieldConstants.shouldFlip else x

    @staticmethod
    def flip_Y_coord(y):
        return FieldConstants.fieldWidth - y if FieldConstants.shouldFlip else y

    @staticmethod
    def flip_Translation2d(translation):
        return (
            Translation2d(
                FieldConstants.flip_X_coord(translation.X()),
                FieldConstants.flip_Y_coord(translation.Y()),
            )
            if FieldConstants.shouldFlip
            else translation
        )

    @staticmethod
    def flip_Rotation2d(rotation):
        return (
            rotation.rotateBy(Rotation2d.fromDegrees(180))
            if FieldConstants.shouldFlip
            else rotation
        )

    @staticmethod
    def flip_Pose2d(pose):
        return (
            Pose2d(
                FieldConstants.flip_Translation2d(pose.translation()),
                FieldConstants.flip_Rotation2d(pose.rotation()),
            )
            if FieldConstants.shouldFlip
            else pose
        )

    class Processor:
        centerFace = Pose2d(inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90))

    class Barge:
        farCage = Translation2d(
            inchesToMeters(345.428), inchesToMeters(286.779)
        )  # cage closest to the middle
        middleCage = Translation2d(inchesToMeters(345.428), inchesToMeters(242.855))
        closeCage = Translation2d(
            inchesToMeters(345.428), inchesToMeters(199.947)
        )  # cage closest to outside wall

        # from floor to bottom of cage
        deepHeight = inchesToMeters(3.125)
        shallowHeight = inchesToMeters(30.125)

    class CoralStation:
        leftCenterFace = Pose2d(
            inchesToMeters(33.526),
            inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011),
        )
        rightCenterFace = Pose2d(
            inchesToMeters(33.526),
            inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90),
        )


    class ReefHeight(Enum):
        L4 = (inchesToMeters(72), -90)
        L3 = (inchesToMeters(47.625), -35)
        L2 = (inchesToMeters(31.875), -35)
        L1 = (inchesToMeters(18), 0)

        def __init__(self, height, pitch):
            self.height = height
            self.pitch = pitch

    class Reef:
        center = Translation2d(inchesToMeters(176.746), inchesToMeters(158.501))
        tag_map = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        faceToZoneLine = inchesToMeters(
            12
        )  # Side of the reef to the inside of the reef zone line
        centerFaces = [
            tag_map.getTagPose(18).toPose2d(),
            tag_map.getTagPose(19).toPose2d(),
            tag_map.getTagPose(20).toPose2d(),
            tag_map.getTagPose(21).toPose2d(),
            tag_map.getTagPose(22).toPose2d(),
            tag_map.getTagPose(17).toPose2d(),
        ]  # Starting facing the driver station in clockwise order
        branchPositions = []

        for face in range(6):
            # Right and left determined from standing outside of the reef looking at the face (not from looking from the inside of reef).
            fillRight = []
            fillLeft = []
            for level in [
                (inchesToMeters(72), -90),
                (inchesToMeters(47.625), -35),
                (inchesToMeters(31.875), -35),
                (inchesToMeters(18), 0),
            ]:
                poseDirection = Pose2d(
                    center, Rotation2d.fromDegrees(180 - (60 * face))
                )
                adjustX = inchesToMeters(30.738)
                adjustY = inchesToMeters(6.469)

                fillRight.append(
                    Pose3d(
                        Translation3d(
                            poseDirection.transformBy(
                                Transform2d(adjustX, adjustY, Rotation2d())
                            ).X(),
                            poseDirection.transformBy(
                                Transform2d(adjustX, adjustY, Rotation2d())
                            ).Y(),
                            level[0],
                        ),
                        Rotation3d(
                            0,
                            degreesToRadians(level[1]),
                            poseDirection.rotation().radians(),
                        ),
                    )
                )
                fillLeft.append(
                    Pose3d(
                        Translation3d(
                            poseDirection.transformBy(
                                Transform2d(adjustX, -adjustY, Rotation2d())
                            ).X(),
                            poseDirection.transformBy(
                                Transform2d(adjustX, -adjustY, Rotation2d())
                            ).Y(),
                            level[0],
                        ),
                        Rotation3d(
                            0,
                            degreesToRadians(level[1]),
                            poseDirection.rotation().radians(),
                        ),
                    )
                )

            branchPositions.append(fillRight)
            branchPositions.append(fillLeft)

    class ReefCalibratedToField:
        """
        JSON in this format:
        {
            "red": {
                "left": {1: (), 2: (), 3: (), 4: (), 5: (), 6: ()},
                "right": {1: (), 2: (), 3: (), 4: (), 5: (), 6: ()}
            },
            "blue": {
                "left": {1: (), 2: (), 3: (), 4: (), 5: (), 6: ()},
                "right": {1: (), 2: (), 3: (), 4: (), 5: (), 6: ()}
            }
        }
        """
        calibrated_data = {
            "red": {
                "left": {1: Pose2d(), 2: Pose2d(), 3: Pose2d(), 4: Pose2d(), 5: Pose2d(), 6: Pose2d()},
                "right": {1: Pose2d(), 2: Pose2d(), 3: Pose2d(), 4: Pose2d(), 5: Pose2d(), 6: Pose2d()}
            },
            "blue": {
                "left": {1: Pose2d(3.166, 4.068, Rotation2d.fromDegrees(90.300)), 2: Pose2d(), 3: Pose2d(), 4: Pose2d(5.827, 3.998, Rotation2d.fromDegrees(-91.32)), 5: Pose2d(5.091, 2.854, Rotation2d.fromDegrees(-149.186)), 6: Pose2d(3.74, 2.899, Rotation2d.fromDegrees(149.85))},
                "right": {1: Pose2d(3.192, 3.798, Rotation2d.fromDegrees(89.766)), 2: Pose2d(), 3: Pose2d(), 4: Pose2d(5.807, 4.3055, Rotation2d.fromDegrees(-91.38)), 5: Pose2d(5.349, 3.029, Rotation2d.fromDegrees(-146.211)), 6: Pose2d(4.05785, 2.74, Rotation2d.fromDegrees(149.35))}
            }
        }

    class StagingPositions:
        """Positions of the starting algae and coral on top of each other"""

        # standing at driver station facing away
        leftIceCream = Translation2d(inchesToMeters(48), inchesToMeters(230.5))
        middleIceCream = Translation2d(inchesToMeters(48), inchesToMeters(158.5))
        rightIceCream = Translation2d(inchesToMeters(48), inchesToMeters(86.5))


# TEST PRINTING FIELD CONST VALUES


## A bunch of constants to test

# print(FieldConstants.Barge.farCage)
# print(FieldConstants.ReefHeight.L4.height)
# print(FieldConstants.Reef.centerFaces[0])
# print(FieldConstants.Reef.branchPositions)
# print(range(len(FieldConstants.Reef.branchPositions)))
# print(len(FieldConstants.Reef.branchPositions))
# print(FieldConstants.Reef.branchPositions)
# print(FieldConstants.Reef.branchPositions)
# print(FieldConstants.Reef.centerFaces)

# #array of elements that are lists of pose3d's for one sector (twelfth (face + right or left branch)) goes top branch to bottom
# # right, left, right, left, ...
# print(FieldConstants.Reef.branchPositions)
# for idx in range(len(FieldConstants.Reef.branchPositions)):
#     for level in range(4):
#         reefHeightLevels = {
#              FieldConstants.ReefHeight.L4 : "4",
#              FieldConstants.ReefHeight.L3 : "3",
#              FieldConstants.ReefHeight.L2 : "2",
#              FieldConstants.ReefHeight.L1 : "1"
#         }
#         for reef_height, lvl in reefHeightLevels.items():
#              if math.isclose(FieldConstants.Reef.branchPositions[idx][level].Z(), reef_height.height, abs_tol=1e-6):
#                 branch_level = lvl

#         print("Face", ((idx // 2) + 1),
#             ", right-branch" if idx % 2 else ", left-branch",
#             ", L" + branch_level,
#             "Pitch:", FieldConstants.Reef.branchPositions[idx][level].rotation().Y(),
#             "\n Pose3d: \n", FieldConstants.Reef.branchPositions[idx][level],
#             end="\n\n"
#             )
