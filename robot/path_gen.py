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
from wpimath.units import inchesToMeters

from field_const import FieldConstants
import math
import numpy as np
from wpilib import SmartDashboard

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
    def __init__(self, lowerLeftCorner : Translation2d, upperRightCorner : Translation2d):
        buffer = 0.3
        x = [lowerLeftCorner.X(), upperRightCorner.X()]
        y = [lowerLeftCorner.Y(), upperRightCorner.Y()]

        self.lowerLeft = Translation2d(min(x) - buffer, min(y) - buffer)
        self.upperRight = Translation2d(max(x) + buffer, max(y) + buffer)

class ObstacleRotation():
    def __init__(self, center : Translation2d, width : float, height : float, rotation : Rotation2d):
        buffer = 0.1
        self.center = center
        self.width = width + 2 * buffer
        self.height = height + 2 * buffer
        self.rotation = rotation


class ObstacleConstants():
    obstacleList = []
    obstacleList.append([Obstacle(Translation2d(inchesToMeters(131.53), inchesToMeters(135.716)), Translation2d(inchesToMeters(221.022), inchesToMeters(187.384))), False]) # 1, 4 faces
    obstacleList.append([ObstacleRotation(Translation2d(inchesToMeters(176.19), inchesToMeters(158.5)), inchesToMeters(89.491), inchesToMeters(51.668), Rotation2d.fromDegrees(-60)), True]) # center, width, height, rotation for 2,5
    obstacleList.append([ObstacleRotation(Translation2d(inchesToMeters(176.19), inchesToMeters(158.5)), inchesToMeters(89.491), inchesToMeters(51.668), Rotation2d.fromDegrees(60)), True]) # 3, 6

class PathGenerator():
    def __init__(self, initialPosition, finalPosition):
        self.initialPosition = FieldConstants.flip_Pose2d(initialPosition)
        self.finalPosition = FieldConstants.flip_Pose2d(finalPosition)
        self.lastSlope = 1
        self.currentSlope = 1
        self.controlPoints = self.buildPath(self.astar(Translation2d(self.initialPosition.X(), self.initialPosition.Y()), Translation2d(self.finalPosition.X(), self.finalPosition.Y())))
        self.all_points = self.getPointList()
        self.smooth_path = self.smooth_points(self.all_points, 0.5, 0.5, 0.0001)
        #self.removeDuplicateSlopes()
        #self.prunePath()

    class PathNode():
        def __init__(self, position, finalPosition, parent=None):
            self.position = position
            self.finalPosition = finalPosition
            self.parent = parent

    def getSmoothPath(self):
        return self.smooth_path

    def containedIn(self, pose : Translation2d, lowerLeft : Translation2d, upperRight : Translation2d) -> bool:
        #lowerleft is on cad default rotation lowerleft
        return (pose.X() >= lowerLeft.X() and pose.Y() >= lowerLeft.Y() and
            pose.X() <= upperRight.X() and pose.Y() <= upperRight.Y())
    def containedInRotated(self, pose : Translation2d, obstacle : ObstacleRotation):
        translated = pose - obstacle.center

        local_x = translated.X() * obstacle.rotation.cos() + translated.Y() * obstacle.rotation.sin()
        local_y = -translated.X() * obstacle.rotation.sin() + translated.Y() * obstacle.rotation.cos()

        return (-obstacle.width / 2 <= local_x <= obstacle.width / 2) and (-obstacle.height / 2 <= local_y <= obstacle.height / 2)



    def inObstacle(self, pose : Translation2d) -> bool:
        for obstacle in ObstacleConstants.obstacleList:
            if obstacle[1]:
                if self.containedInRotated(pose, obstacle[0]):
                    return True
            else:
                if self.containedIn(pose, obstacle[0].lowerLeft, obstacle[0].upperRight):
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
                pose = Translation2d(node.position.X() + (x / 2), node.position.Y() + (y / 2))
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


    def smooth_points(self, path : list, weight_smoothing, weight_data, tolerance):
        newPath = path
        change = tolerance
        
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path) - 1):
                x = newPath[i].X()
                y = newPath[i].Y()
                aux_x = x
                aux_y = y

                new_x = x + weight_smoothing * (path[i].X() - x) + weight_data * (newPath[i - 1].X() + newPath[i + 1].X() - 2.0 * x)
                new_y = y + weight_smoothing * (path[i].Y() - y) + weight_data * (newPath[i - 1].Y() + newPath[i + 1].Y() - 2.0 * y)
                if self.obstacleBetween(Translation2d(aux_x, aux_y), Translation2d(new_x, new_y)):
                    continue
                else:
                    newPath[i] = Translation2d(new_x, new_y)
                change += abs(aux_x - new_x) + abs(aux_y - new_y)
        return newPath
    
class PurePursuitController():
    def __init__(self, lookahead_dist, smooth_path):
        self.last_closest_point_idx = 0
        self.lookahead_dist = lookahead_dist
        self.last_lookahead_point_idx = 0
        self.last_lookahead_point = None
        self.path = smooth_path

    def getClosestPoint(self, curPose : Pose2d, start_idx : int) -> Translation2d:
        min_dist = math.inf
        print("len path: ", len(self.path))
        for i in range(start_idx, len(self.path) - 2):
            dist = (self.path[i] - curPose.translation()).norm()
            if dist < min_dist:
                min_dist = dist
                start_idx = i
        print("closest idx: ", start_idx) # Ensure progress
        self.last_closest_point_idx = start_idx
        return [self.path[start_idx], start_idx]

        
        
        # path = self.path
        # closest_point = path[0]
        # closest_point_idx = 0 # default
        # for idx in range(len(path)):
        #     if (curPose.translation() - path[idx]).norm() <= (curPose.translation() - closest_point).norm():
        #         closest_point = path[idx]
        #         closest_point_idx = idx
        # self.last_closest_point_idx = closest_point_idx
        # return [closest_point, closest_point_idx]
    
    def getLookaheadIntersectionAllPath(self, curPose: Pose2d):
        best_lookahead = curPose.translation()
        best_alignment = -1
        robot_heading = curPose.rotation()
        robot_direction = Translation2d(robot_heading.cos(), robot_heading.sin())
        
        for idx in range(self.getClosestPoint(curPose, self.last_closest_point_idx)[1] + 1, len(self.path) - 2):
            intersections = self.getLookaheadIntersection(curPose, idx)
            #print(intersections)
            if intersections:
                for lookahead in intersections:
                    path_segment = (self.path[idx  + 1] - self.path[idx])
                    alignment = robot_direction.X() * path_segment.X() + robot_direction.Y() * path_segment.Y()

                    if alignment > best_alignment:
                        best_alignment = alignment
                        best_lookahead = lookahead
            if best_lookahead == curPose.translation():
                continue
            else:  
                self.last_lookahead_point = best_lookahead
                self.last_lookahead_point_idx = idx
                break
        else:
            best_lookahead = self.path[self.getClosestPoint(curPose, 0)[1] + 1]
        
        print("best lookahead: ", best_lookahead)
        return best_lookahead

            

    def getLookaheadIntersection(self, curPose : Pose2d, start_point_idx : int):
        path = self.path
        start_point = path[start_point_idx]
        end_point = path[start_point_idx + 1]

        direction_vector = end_point - start_point
        pose_to_start_point = start_point - curPose.translation()

        a = (direction_vector.X() ** 2) + (direction_vector.Y() ** 2)  # Squared magnitude of d
        b = 2 * (pose_to_start_point.X() * direction_vector.X() + pose_to_start_point.Y() * direction_vector.Y())  # Interaction between d and f
        c = ((pose_to_start_point.X() ** 2) + (pose_to_start_point.Y() ** 2)) - (self.lookahead_dist ** 2)  # Determines circle intersection condition

        discriminant = (b ** 2) - (4 * a * c)
        if discriminant < 0:
            return False
        discriminant = math.sqrt(discriminant)
        
        intersections = []
        candidate_intersection_1 = (-b - discriminant) / (2 * a)
        candidate_intersection_2 = (-b + discriminant) / (2 * a) # because quadratic equation is plus-minus

        
        
        # if 0 <= candidate_intersection_1 <= 1:
        #     print("intersect 1")
        #     intersections.append(Translation2d(start_point.X() + candidate_intersection_1 * direction_vector.X(), start_point.Y() + candidate_intersection_1 * direction_vector.Y()))
        # if 0 <= candidate_intersection_2 <= 1:
        #     print("intersect 2")
        #     intersections.append(Translation2d(start_point.X() + candidate_intersection_2 * direction_vector.X(), start_point.Y() + candidate_intersection_2 * direction_vector.Y()))
        for candidate in [candidate_intersection_1, candidate_intersection_2]:
            if 0 <= candidate <= 1:
                intersection = Translation2d(
                    start_point.X() + candidate * direction_vector.X(),
                    start_point.Y() + candidate * direction_vector.Y()
                )

                # Ensure forward progress (dot product check)
                lookahead_direction = intersection - curPose.translation()
                if (lookahead_direction.X() * direction_vector.X() + lookahead_direction.Y() * direction_vector.Y()) > 0:
                    intersections.append(intersection)


        if intersections == []:
            return False
        else:
            print("have intersections: ", intersections)
            return intersections
    def isPosesClose(self, pose1 : Translation2d, pose2 : Translation2d):
        if (pose1 - pose2).norm() < 0.05:
            return True
        else:
            return False

    def getVelocities(self, curPose : Pose2d):
        lookahead_point = self.getLookaheadIntersectionAllPath(curPose)
        if self.isPosesClose(curPose.translation(), self.path[-1]):
            print("end path")
            return False
        vx = lookahead_point.X() - curPose.X()
        vy = lookahead_point.Y() - curPose.Y()
        SmartDashboard.putNumber("vx velocity", vx)
        SmartDashboard.putNumber("vy velocity", vy)
        return [vx, vy]