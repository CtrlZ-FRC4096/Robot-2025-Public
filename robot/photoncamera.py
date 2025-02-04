import wpilib
from wpimath.units import feetToMeters
from photonlibpy.photonCamera import (
    PhotonCamera,
    setVersionCheckEnabled,
)  # VisionLEDMode
from photonlibpy import photonPoseEstimator
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Translation2d,
    Translation3d,
    Rotation2d,
    Rotation3d,
)

import const
from wpilib import DriverStation, SmartDashboard, Timer, Field2d

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from field_const import FieldConstants

import numpy as np
import math
import cv2


## Code from 1736
# Describes one on-field pose estimate from the a camera at a specific time.
class CameraPoseObservation:
    def __init__(self, time, estFieldPose, trustworthiness=1.0):
        self.time = time
        self.estFieldPose = estFieldPose
        self.trustworthiness = trustworthiness  # TODO - not used yet


# Wrappers photonvision to:
# 1 - resolve issues with target ambiguity (two possible poses for each observation)
# 2 - Convert pose estimates to the field
# 3 - Handle recording latency of when the image was actually seen
class WrapperedPhotonCamera:
    def __init__(self, camName, robotToCam):
        # setVersionCheckEnabled(False)

        self.cam = PhotonCamera(camName)
        # TODO is this really the name of the camera or is this just as a reminder? Camera1,2,3,or 4??
        self.cameraDistortVector = const.CAM_DICT[camName][0]
        self.cameraIntrinsMatrix = const.CAM_DICT[camName][1]

        self.timeoutSec = 1.0
        self.poseEstimates = []
        self.robotToCam = robotToCam
        self.counter = 0

    def update(self, prevEstPose: Pose2d, allianceColor: str):
        # self.counter += 1
        self.poseEstimates = []
        self.tagPositions = []
        self.tagAmbiguity = []
        self.poseSingleTag = []
        self.singleTagIDs = []
        # if (self.counter % 20 == 0):
        #     if not self.cam.isConnected():
        #         # Faulted - no estimates, just return.
        #         print("Camera not connected")
        #        pass
        #        return

        # Grab whatever the camera last reported for observations in a camera frame
        # Note: Results simply report "I processed a frame". There may be 0 or more targets seen in a frame
        res = self.cam.getLatestResult()

        # MiniHack - results also have a more accurate "getTimestamp()", but this is
        # broken in photonvision 2.4.2. Hack with the non-broken latency calcualtion
        # latency = res.getLatencyMillis()
        # obsTime = wpilib.Timer.getFPGATimestamp() - latency

        obsTime = res.getTimestampSeconds()

        ## MultiTag code
        tag_map = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)
        photon_pose_estimator = photonPoseEstimator.PhotonPoseEstimator(
            tag_map,
            photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.cam,
            self.robotToCam,
        )
        vision_est = photon_pose_estimator.update(res)

        if vision_est is not None:

            robot_pose = vision_est.estimatedPose.toPose2d()

            # if ((robot_pose.x > -0.5) and (robot_pose.x < const.FIELD_LENGTH_METERS + 0.5) and (robot_pose.y > -0.5) and (robot_pose.y < const.FIELD_WIDTH_METERS + 0.5)): # Check if the robot is on the field
            self.poseEstimates.append(CameraPoseObservation(obsTime, robot_pose))
            for target in res.getTargets():
                tgtID = target.getFiducialId()

                tagFieldPose = tag_map.getTagPose(tgtID)
                self.tagAmbiguity.append(target.getPoseAmbiguity())
                self.tagPositions.append(tagFieldPose)

        ## Single Tag Code
        # Process each target.
        # Each target has multiple solutions for where you could have been at on the field
        # when you observed it
        # (https://docs.wpilib.org/en/stable/docs/software/vision-processing/
        # apriltag/apriltag-intro.html#d-to-3d-ambiguity)
        # We want to select the best possible pose per target
        # We should also filter out targets that are too far away, and poses which
        # don't make sense.

        tag_map = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

        for target in res.getTargets():

            # Transform both poses to on-field poses
            tgtID = target.getFiducialId()
            if tgtID in [
                6,
                7,
                8,
                9,
                10,
                11,
                17,
                18,
                19,
                20,
                21,
                22,
            ]:  # Only use reef IDs, everything else is not great

                tagFieldPose = tag_map.getTagPose(tgtID)

                corners = np.array(
                    target.getDetectedCorners()
                )  # Return list of n corners, for fiducials this is counter clockwise starting from the top left corner of the tag.
                corners_undistorted = cv2.undistortPoints(  # Unsure if these corners have already been undistorted
                    corners,
                    self.cameraIntrinsMatrix,
                    self.cameraDistortVector,
                )  # Return list of n corners, for fiducials this is counter clockwise starting from the top left corner of the tag.

                corners = np.zeros((4, 2))
                for index, corner in enumerate(
                    corners
                ):  # calculate the angle of each corner relative to the camera center in the x and y directions (radians)
                    vec = np.linalg.inv(self.cameraIntrinsMatrix).dot(
                        np.array([corner[0][0], corner[0][1], 1]).T
                    )
                    corners[index][0] = math.atan(vec[0])
                    corners[index][1] = math.atan(vec[1])

                # Calculate the center of the target in x and y angles (radians)
                target_x_angle = np.mean(corners[:, 0])
                target_y_angle = np.mean(corners[:, 1])

                distance = (
                    target.getBestCameraToTarget().translation().norm()
                )  # distance from camera to target in meters

                # Calculate the position of the target to the camera  in the camera coordinate system (meters)
                # Use spherical coordinates to calculate the x, y, and z distances
                z_dist = distance * math.cos((math.pi / 2) - target_y_angle)
                y_dist = (
                    distance
                    * math.sin(target_x_angle)
                    * math.sin((math.pi / 2) - target_y_angle)
                )
                x_dist = (
                    distance
                    * math.cos(target_x_angle)
                    * math.sin((math.pi / 2) - target_y_angle)
                )

                camToTarget = Pose3d(
                    Translation3d(x_dist, y_dist, z_dist), Rotation3d()
                )  # Create a Pose3d object with the calculated x, y, and z distances, and no rotation

                # Calculate the position of the robot on the field in the field coordinate system (meters) from the tag pose and the camera to target transform
                fieldPose = self._toFieldPose(tagFieldPose, camToTarget)

                self.poseSingleTag.append(fieldPose)
                self.singleTagIDs.append(tgtID)

    def getTagIds(self):
        return self.tag_ids

    def getPoseEstimates(self):
        return self.poseEstimates

    def getTagPositions(self):
        return self.tagPositions

    def getTagAmbiguity(self):
        return self.tagAmbiguity

    def getPoseSingleTag(self):
        return self.poseSingleTag

    def getSingleTagIDs(self):
        return self.singleTagIDs

    def _toFieldPose(self, tgtPose, camToTarget):
        camPose = tgtPose.transformBy(camToTarget.inverse())
        return camPose.transformBy(self.robotToCam.inverse()).toPose2d()

    # Returns true of a pose is on the field, false if it's outside of the field perimieter
    @staticmethod
    def _poseIsOnField(self, pose: Pose2d):
        trans = pose.translation()
        x = trans.X()
        y = trans.Y()
        inY = -0.5 < y < FieldConstants.fieldWidth + 0.5
        inX = -0.5 < x < FieldConstants.fieldLength + 0.5
        return inX and inY
