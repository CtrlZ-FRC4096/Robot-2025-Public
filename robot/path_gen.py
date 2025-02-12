from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import (
    Pose2d,
    Translation2d,
)
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint

from field_const import FieldConstants

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

class PathGenerator():
    def __init__(self, initialPosition, finalPosition):
        self.initialPosition = FieldConstants.flip_Pose2d(initialPosition)
        self.finalPosition = FieldConstants.flip_Pose2d(finalPosition)
        self.lastSlope = 1
        self.currentSlope = 1
        self.controlPoints = self.buildPath(self.astar(Translation2d(self.initialPosition.X(), self.initialPosition.Y()), Translation2d(self.finalPosition.X(), self.finalPosition.Y())))
        # self.removeDuplicateSlopes()
        # self.prunePath()

    class PathNode():
        def __init__(self, position, finalPosition, parent=None):
            self.position = position
            self.finalPosition = finalPosition
            self.parent = parent
    def containedIn(self, pose : Translation2d, lowerLeft : Translation2d, upperRight : Translation2d) -> bool:
        return pose.X() >= lowerLeft.X() and pose.Y() >= lowerLeft.Y() and pose.X() <= upperRight.X() and pose.Y() <= upperRight.Y()

    def inObstacle(self, pose : Translation2d) -> bool:
        return False #TODO: ADD OBSTACLES

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
        slope = (first.Y() - second.Y()) / (first.X() - second.X())
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
