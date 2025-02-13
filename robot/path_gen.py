from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import (
    Pose2d,
    Translation2d,
    Rotation2d,
)
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModuleState,
)
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint

from field_const import FieldConstants
import math

class QueueNode():
    def __init__(self, data, cost):
        self.data = data
        self.cost = cost

class PriorityQueue():
    def __init__(self):
        self.nodes = []

    def add(self, data, cost):
        self.nodes.append(QueueNode(data, cost))

    def remove(self):
        lowest = self.nodes[0].cost
        lowestIndex = 0

        for i in range(len(self.nodes)):
            if self.nodes[i].cost < lowest:
                lowest = self.nodes[i].cost
                lowestIndex = i

        return self.nodes.pop(lowestIndex).data
    def isEmpty(self):
        return len(self.nodes) == 0

class Obstacle():
    def __init__(self, lowerLeftCorner, upperRightCorner):
        buffer = 0.3
        x = [lowerLeftCorner.X(), upperRightCorner.X()]
        y = [lowerLeftCorner.Y(), upperRightCorner.Y()]

        self.lowerLeft = Translation2d(min(x) - buffer, min(y) - buffer)
        self.upperRight = Translation2d(max(x) + buffer, max(y) + buffer)

class ObstacleConstants():
    obstacleList = []
    obstacleList.append(Obstacle(Translation2d(3.0, 5.0), Translation2d(5.0, 3.0)))
    #1-4 lowerLeft inches (131.53, 135.716), upperRight inches (221.022, 187.384)
    #2-5
    #3-6

class PathGenerator():
    def __init__(self, initialPosition, finalPosition):
        self.initialPosition = FieldConstants.flip_Pose2d(initialPosition)
        self.finalPosition = FieldConstants.flip_Pose2d(finalPosition)
        self.lastSlope = 1
        self.currentSlope = 1
        self.controlPoints = self.buildPath(self.astar(Translation2d(self.initialPosition.X(), self.initialPosition.Y()), Translation2d(self.finalPosition.X(), self.finalPosition.Y())))
        #self.removeDuplicateSlopes()
        #self.prunePath()

    class PathNode():
        def __init__(self, position, finalPosition, parent=None):
            self.position = position
            self.finalPosition = finalPosition
            self.parent = parent

    def containedIn(self, pose : Translation2d, lowerLeft : Translation2d, upperRight : Translation2d) -> bool:
        #lowerleft is on cad default rotation lowerleft
        return (pose.X() >= lowerLeft.X() and pose.Y() >= lowerLeft.Y() and
            pose.X() <= upperRight.X() and pose.Y() <= upperRight.Y()
            )

    def inObstacle(self, pose : Translation2d) -> bool:
        for obstacle in ObstacleConstants.obstacleList:
            if self.containedIn(pose, obstacle.lowerLeft, obstacle.upperRight):
                return True
        return False

    def obstacleBetween(self, initialPose : Translation2d, finalPose : Translation2d):
        steps = 25
        step = Translation2d((finalPose.X() - initialPose.X()) / steps,  (finalPose.Y() - initialPose.Y()) / steps)
        for _ in range(steps):
            if self.inObstacle(initialPose):
                return True
            initialPose = initialPose.__add__(step)
        return False

    def getNeighbors(self, node : PathNode, finalPosition : Translation2d):
        neighbors = []
        for x in range(-1, 2):
            for y in range(-1, 2):
                if x == y:
                    continue
                pose = Translation2d(node.position.X() + (x / 4), node.position.Y() + (y / 4))
                if not(self.inObstacle(pose)):
                    element = self.PathNode(pose, finalPosition)
                    neighbors.append(element)
        return neighbors

    def astar(self, initialPosition : Translation2d, finalPosition : Translation2d):
        frontier = PriorityQueue()
        frontier.add(self.PathNode(initialPosition, finalPosition), 0)
        visited = []
        while not(frontier.isEmpty()):
            currentNode = frontier.remove()
            if (currentNode.position - finalPosition).norm() < 1:
                return currentNode
            for child in self.getNeighbors(currentNode, finalPosition):
                if not(child.position in visited):
                    visited.append(child.position)
                    child.parent = currentNode
                    frontier.add(child, (child.position - finalPosition).norm() + (child.position - initialPosition).norm())
        return None
    def buildPath(self, finalNode : PathNode | None):
        path = []
        currentNode = finalNode
        while currentNode.parent != None:
            path.append(currentNode.position)
            currentNode = currentNode.parent
        return path

    def getSlope(self, first : Translation2d, second : Translation2d):
        if (first.X() - second.X()) == 0:
            return self.lastSlope
        slope = abs(first.Y() - second.Y()) / abs(first.X() - second.X())
        return slope

    def removeDuplicateSlopes(self):
        if len(self.controlPoints) < 2:
            return
        newPath = []
        self.lastSlope = self.getSlope(self.initialPosition.translation(), self.controlPoints[1])
        for idx in range(len(self.controlPoints) - 1):
            self.currentSlope = self.getSlope(self.controlPoints[idx], self.controlPoints[idx + 1])
            if self.currentSlope != self.lastSlope:
                newPath.append(self.controlPoints[idx])
            self.lastSlope = self.currentSlope
        self.controlPoints = newPath

    def prunePath(self):
        for i in reversed(range(len(self.controlPoints))):
            if not(self.obstacleBetween(self.initialPosition.translation(), self.controlPoints[i])):
                del self.controlPoints[0:i]
                break
        for i in range(len(self.controlPoints)):
            if not(self.obstacleBetween(self.finalPosition.translation(), self.controlPoints[i])):
                del self.controlPoints[i+1:]
                break

    def getPointList(self):
        points = self.controlPoints
        points.insert(0, self.initialPosition.translation())
        points.append(self.finalPosition.translation())
        return points

# class PurePursuit():
#     def __init__(self, path, lookahead_dist, max_speed, swerve_kinematics):
#         """
#         :param path: List of Translation2d waypoints.
#         :param lookahead_distance: Distance to look ahead on the path.
#         :param max_speed: Maximum robot speed (m/s).
#         :param swerve_kinematics: WPILib SwerveDriveKinematics object.
#         """
#         self.path = path
#         self.lookahead_dist = lookahead_dist
#         self.max_spede = max_speed
#         self.kinematics = swerve_kinematics
#     def find_lookahead_point(self, curPose : Pose2d):
#         for point in self.path:
#             if (curPose.translation() - point).norm() >= self.lookahead_dist:
#                 return point
#         else:
#             return self.path[-1]
        
#     def calculate_chassis_speeds(self, curPose : Pose2d):
#         lookahead = self.find_lookahead_point(curPose)
#         relative_lookahead : Translation2d = lookahead - curPose.translation()
#         heading = curPose.rotation()

#         xL = relative_lookahead.rotateBy(heading.__neg__()).X()
#         yL = relative_lookahead.rotateBy(heading.__neg__()).Y()

#         # Compute curvature
#         if yL == 0:
#             curvature = 0  # Drive straight
#         else:
#             curvature = (2 * yL) / (self.lookahead_distance ** 2)

#         # Convert curvature into desired speeds
#         vx = self.max_speed  # Forward velocity
#         omega = curvature * self.max_speed  # Rotational velocity

#         return ChassisSpeeds(vx, 0, omega)
#     def curvature_to_point(self, curEstPose: Pose2d, point: Translation2d):
#         x_slope = -math.tan(position.Theta())
#         y_slope = 1
#         y_intersect = math.tan(position.Theta()) * position.X() - position.Y()

#         # Calculate perpendicular distance from the point to the line
#         x = abs(point.X() * x_slope + point.Y() * b + c) / math.sqrt(a * a + b * b)

#         # Calculate side of the line (left or right)
#         side_l = math.sin(position.Theta()) * (point.X() - position.X()) - math.cos(position.Theta()) * (point.Y() - position.Y())
#         side = side_l / abs(side_l) if side_l != 0 else 0  # side is either 1 or -1, or 0 if exactly on the line

#         if side_l == 0:
#             return 0  # Curvature is 0 if point is exactly on the line

#         # Calculate chord (distance between robot and point)
#         chord = math.sqrt((point.X() - position.X())**2 + (point.Y() - position.Y())**2)

#         # Calculate and return the curvature
#         return (2 * x) / (chord ** 2) * side