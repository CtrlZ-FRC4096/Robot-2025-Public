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

# class Obstacle():
#     def __init__(self, lowerLeftCorner : Translation2d, upperRightCorner : Translation2d):
#         buffer = 0.5
#         x = [lowerLeftCorner.X(), upperRightCorner.X()]
#         y = [lowerLeftCorner.Y(), upperRightCorner.Y()]

#         self.lowerLeft = Translation2d(min(x) - buffer, min(y) - buffer)
#         self.upperRight = Translation2d(max(x) + buffer, max(y) + buffer)

class ObstacleRotation():
    def __init__(self, center : Translation2d, width : float, height : float, rotation : Rotation2d):
        buffer = 0.5
        self.center = center
        self.width = width + 2 * buffer
        self.height = height + 2 * buffer
        self.rotation = rotation


class ObstacleConstants():
    buffer = 3
    reefVertices = [
        Translation2d(3.643, 3.565),
        Translation2d(3.654, 4.519),
        Translation2d(4.486, 4.986),
        Translation2d(5.307, 4.501),
        Translation2d(5.298, 3.547),
        Translation2d(4.467, 3.077)
    ] #face's right branch point
    for idx in range(len(reefVertices)):
        reefVertices[idx] = Translation2d(reefVertices[idx].X() + buffer, reefVertices[idx].Y() + buffer)
    #obstacleList = [Obstacle(Translation2d(3.0, 3.0), Translation2d(4.5, 4.5))]


class PathGenerator():
    def __init__(self, initialPosition, finalPosition):
        self.initialPosition = FieldConstants.flip_Pose2d(initialPosition)
        self.finalPosition = FieldConstants.flip_Pose2d(finalPosition)
        self.lastSlope = 1
        self.currentSlope = 1
        self.controlPoints = self.buildPath(self.astar(Translation2d(self.initialPosition.X(), self.initialPosition.Y()), Translation2d(self.finalPosition.X(), self.finalPosition.Y())))
        #self.removeDuplicateSlopes()

        self.all_points = self.getPointList()
        self.smooth_path = self.smooth_points(self.all_points, 0.5, 0.5, 1)
        # self.prunePath()




    class PathNode():
        def __init__(self, position, finalPosition, parent=None):
            self.position = position
            self.finalPosition = finalPosition
            self.parent = parent

    def getSmoothPath(self):

        return self.smooth_path

    # def containedIn(self, pose : Translation2d, lowerLeft : Translation2d, upperRight : Translation2d) -> bool:
    #     #lowerleft is on cad default rotation lowerleft
    #     return (pose.X() >= lowerLeft.X() and pose.Y() >= lowerLeft.Y() and
    #         pose.X() <= upperRight.X() and pose.Y() <= upperRight.Y())


    def inReef(self, pose: Translation2d):
        x = pose.X()
        y = pose.Y()
        x_navgrid = math.floor(x / 0.3)
        y_navgrid = math.floor(y / 0.3)
        #8 - 18 in y
        inReef = False
        if x_navgrid == 10 or x_navgrid == 19:
            if y_navgrid in range(11,16):
                inReef = True
            else:
                inReef = False
        elif x_navgrid == 11 or x_navgrid == 18:
            if y_navgrid in range(10,17):
                inReef = True
            else:
                inReef = False
        elif x_navgrid == 12 or x_navgrid == 13 or x_navgrid == 16 or x_navgrid == 17:
            if y_navgrid in range(9, 18):
                inReef  = True
            else:
                inReef = False
        elif x_navgrid == 14 or x_navgrid == 15:
            if y_navgrid in range(8, 19):
                inReef =  True
            else:
                inReef = False
        else:
            inReef = False
        return inReef




    # def containedInRotated(self, pose : Translation2d, obstacle : ObstacleRotation):
    #     translated = pose - obstacle.center

    #     local_x = translated.X() * obstacle.rotation.cos() + translated.Y() * obstacle.rotation.sin()
    #     local_y = -translated.X() * obstacle.rotation.sin() + translated.Y() * obstacle.rotation.cos()

    #     return (-obstacle.width / 2 <= local_x <= obstacle.width / 2) and (-obstacle.height / 2 <= local_y <= obstacle.height / 2)

    def inObstacle(self, pose : Translation2d) -> bool:
        if self.inReef(pose):
                return True
        return False

    def obstacleBetween(self, initialPose : Translation2d, finalPose : Translation2d):
        steps = 50
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
                #if x == y:
                 #   continue
                pose = Translation2d(node.position.X() + (x / 20), node.position.Y() + (y / 20))
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
        for i in reversed(range(len(self.smooth_path))):
            if not(self.obstacleBetween(self.initialPosition.translation(), self.smooth_path[i])):
                del self.smooth_path[0:i]
                break
        for i in range(len(self.smooth_path)):
            if not(self.obstacleBetween(self.finalPosition.translation(), self.smooth_path[i])):
                del self.smooth_path[i+1:]
                break

    def getPointList(self):
        points = [self.initialPosition.translation()] + self.controlPoints + [self.finalPosition.translation()]
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

    def getClosestPoint(self, curPose : Pose2d, start_idx : int):
        min_dist = math.inf
        # print("len path: ", len(self.path))
        for i in range(start_idx, len(self.path) - 2):
            dist = (self.path[i] - curPose.translation()).norm()
            if dist < min_dist:
                min_dist = dist
                start_idx = i
        # print("closest idx: ", start_idx) # Ensure progress
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
            print("no intersections")
            best_lookahead = self.path[self.getClosestPoint(curPose, 0)[1] + 1] # SHOULDN'T NEED THIS

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
            # print("have intersections: ", intersections)
            return intersections
    def isAtEnd(self, curPose : Pose2d):
        if (curPose.translation() - self.path[-1]).norm() < 0.3: #if we are closer than 0.3 meters
            return True
        return False

    def getVelocities(self, curPose : Pose2d):
        # lookahead_point = self.getLookaheadIntersectionAllPath(curPose)

        if self.isAtEnd(curPose):
            return False
        try:
            lookahead_point = self.path[self.getClosestPoint(curPose, 0)[1] + 10]
        except:
            lookahead_point = self.path[self.getClosestPoint(curPose, 0)[1] + 1]
        vx = lookahead_point.X() - curPose.X()
        vy = lookahead_point.Y() - curPose.Y()
        print("in velocities")
        SmartDashboard.putNumber("vx velocity", vx)
        SmartDashboard.putNumber("vy velocity", vy)
        return [vx, vy]