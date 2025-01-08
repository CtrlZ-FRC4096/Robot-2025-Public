import wpilib
from wpimath.units import feetToMeters
from photonlibpy.photonCamera import (
    PhotonCamera,
    setVersionCheckEnabled,
)  # VisionLEDMode
from photonlibpy import photonPoseEstimator
from wpimath.geometry import Pose2d

import const
from wpilib import DriverStation, SmartDashboard, Timer, Field2d

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


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

        self.timeoutSec = 1.0
        self.poseEstimates = []
        self.robotToCam = robotToCam
        self.counter = 0

    def update(self, prevEstPose: Pose2d, allianceColor: str):
        # self.counter += 1
        self.poseEstimates = []
        self.tagPositions = []
        self.tagAmbiguity = []
        self.saw_speaker_tag = False  # speaker tag: 4 for red, 7 for blue
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

        # Update our disconnected fault since we have something from the camera

        # Process each target.
        # Each target has multiple solutions for where you could have been at on the field
        # when you observed it
        # (https://docs.wpilib.org/en/stable/docs/software/vision-processing/
        # apriltag/apriltag-intro.html#d-to-3d-ambiguity)
        # We want to select the best possible pose per target
        # We should also filter out targets that are too far away, and poses which
        # don't make sense.

        ## Previous 1 target code
        # for target in res.getTargets():

        #     # Transform both poses to on-field poses
        #     tgtID = target.getFiducialId()
        #     # if tgtID in [
        #     #     3,
        #     #     4,
        #     #     7,
        #     #     8,
        #     # ]:  # Only use speaker IDs, everything else is not great
        #         # Only handle valid ID's
        #     tagFieldPose = loadAprilTagLayoutField(
        #         AprilTagField.k2024Crescendo
        #     ).getTagPose(tgtID)

        #     if tagFieldPose is not None:
        #         # Only handle known tags
        #         poseCandidates: list[Pose2d] = []
        #         poseCandidates.append(
        #             self._toFieldPose(tagFieldPose, target.getBestCameraToTarget())
        #         )
        #         poseCandidates.append(
        #             self._toFieldPose(
        #                 tagFieldPose, target.getAlternateCameraToTarget()
        #             )
        #         )

        #         # Filter candidates in this frame to only the valid ones
        #         filteredCandidates: list[Pose2d] = []
        #         for candidate in poseCandidates:
        #             onField = self._poseIsOnField(candidate)
        #             # Add other filter conditions here
        #             if onField:
        #                 filteredCandidates.append(candidate)

        #         # Pick the candidate closest to the last estimate
        #         bestCandidate: Pose2d | None = None
        #         bestCandidateDist = 99999999.0
        #         for candidate in filteredCandidates:
        #             delta = (candidate - prevEstPose).translation().norm()
        #             if delta < bestCandidateDist:
        #                 # This candidate is better, use it
        #                 bestCandidate = candidate
        #                 bestCandidateDist = delta

        #         # Finally, add our best candidate the list of pose observations
        #         if bestCandidate is not None:
        #             self.poseEstimates.append(
        #                 CameraPoseObservation(obsTime, bestCandidate)
        #             )
        #             self.tagAmbiguity.append(target.getPoseAmbiguity())
        #             self.tagPositions.append(tagFieldPose)

        ## MultiTag code
        tag_map = AprilTagFieldLayout([], const.FIELD_LENGTH_METERS, const.FIELD_WIDTH_METERS)
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

                if (allianceColor == DriverStation.Alliance.kRed and tgtID == 4) or (
                    allianceColor == DriverStation.Alliance.kBlue and tgtID == 7
                ):
                    self.saw_speaker_tag = True

                tagFieldPose = AprilTagFieldLayout(
                    [], const.FIELD_LENGTH_METERS, const.FIELD_WIDTH_METERS
                ).getTagPose(tgtID)
                self.tagAmbiguity.append(target.getPoseAmbiguity())
                self.tagPositions.append(tagFieldPose)

    def getTagIds(self):
        return self.tag_ids

    def getPoseEstimates(self):
        return self.poseEstimates

    def getTagPositions(self):
        return self.tagPositions

    def getTagAmbiguity(self):
        return self.tagAmbiguity

    def _toFieldPose(self, tgtPose, camToTarget):
        camPose = tgtPose.transformBy(camToTarget.inverse())
        return camPose.transformBy(self.robotToCam.inverse()).toPose2d()

    # Returns true of a pose is on the field, false if it's outside of the field perimieter
    def _poseIsOnField(self, pose: Pose2d):
        trans = pose.translation()
        x = trans.X()
        y = trans.Y()
        inY = 0.0 <= y <= feetToMeters(27.0)
        inX = 0.0 <= x <= feetToMeters(54.0)
        return inX and inY
