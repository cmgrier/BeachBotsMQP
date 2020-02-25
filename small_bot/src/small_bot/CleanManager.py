from data.Task import Task
from support.Constants import *
import geometry_msgs.msg
from navigation.AStar import AStar
from geometry_msgs.msg import Pose
import math

class CleanManager:

    def __init__(self, smallbot, taskManager):
        self.smallbot = smallbot
        self.taskManager = taskManager
        self.counter = 0    # here for testing
        self.current_task_id = -1
        self.waypoints = []
        self.testing = True
        pass

    def do_task(self, task):
        # this will attempt to complete the given clean task,
        # update progress on task and return the updated task
        if self.testing:
            return self.do_task_test(task)
        if self.current_task_id != task.zone.id:
            self.current_task_id = task.zone.id
            self.waypoints = self.make_path_in_zone(task)
        elif len(self.waypoints) > 0:
            if self.is_at_position(self.waypoints[0]):
                self.waypoints.pop(0)
            else:
                self.nav_to_point(self.waypoints[0])

    def do_task_test(self, task):
        self.counter = self.counter + 1
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 60
        self.taskManager.pub_vel(twist)

        if DEBUG:
            print("working on cleaning task")
            print(self.counter)
        if self.counter > 100:
            path = self.nav_to_zone(task, self.smallbot.occupancyGrid)
            print("path:")
            print(path)
            print("starting point")
            print(path[0])
            print("CLEANING TASK COMPLETE")
            self.counter = 0
            task.isComplete = True
        return task

    # returns True if this robot is at the given pose
    def is_at_position(self, pose):
        x = self.smallbot.position.position.x
        y = self.smallbot.position.position.y
        return pose.position.x - POSITION_THRESHOLD < x < pose.position.x + POSITION_THRESHOLD \
            and pose.position.y - POSITION_THRESHOLD < y < pose.position.y + POSITION_THRESHOLD

    # tell robot to drive to a certain pose in a straight line
    def nav_to_point(self, pose):
        # do the thing
        pass

    # makes a path in zone
    def make_path_in_zone(self, task):
        waypoints = []
        tl = task.zone.corners[0]
        tr = task.zone.corners[1]
        br = task.zone.corners[2]
        bl = task.zone.corners[3]
        x_change = tl.position.x - bl.position.x
        y_change = tl.position.y - tr.position.y
        start = Pose()
        start.position.x = (br.position.x + bl.position.x) / 2.0
        start.position.y = (br.position.y + bl.position.y) / 2.0
        waypoints.append(start)
        y_pos = start.position.y
        y_max = (tl.position.y + tr.position.y) / 2.0
        y_increment = (y_max - y_pos) / WAYPOINT_DENSITY
        left = True
        while y_pos < y_max:
            point = Pose()
            if left:
                point.position.x = start.position.x - ZONE_WIDTH / 4.0
            else:
                point.position.x = start.position.x + ZONE_WIDTH / 4.0
            point.position.y = start.position.y + y_increment
            waypoints.append(point)
            if left:
                left = False
            else:
                left = True
            y_pos += y_increment
        end_point = Pose()
        end_point.position.x = (tl.position.x + tr.position.x) / 2.0
        end_point.position.y = (tl.position.y + tr.position.y) / 2.0
        waypoints.append(end_point)
        return waypoints

    # makes a path of waypoints to get to the beginning of the zone
    def nav_to_zone(self, task, OG):
        if len(OG.data) == 0:
            return [self.smallbot.position]
        astar = AStar(OG.data)
        # calculate these with given task
        start = self.point_to_index(self.smallbot.position.position, OG)
        goal = self.point_to_index(task.start_point.position, OG)
        path = astar.find_path(start, goal)
        points = self.get_path_of_points(path, OG)
        return points

    # converts a list of indexes to a list of Poses
    def get_path_of_points(self, path_of_indexes, OG):
        print(path_of_indexes)
        pp = []
        for index in path_of_indexes:
            pp.append(self.index_to_point(index, OG))
        return pp

    # converts an index to a Pose
    def index_to_point(self, index, OG):
        print("Index: " + str(index))
        x_change = (X_MAX * 2) / OG_WIDTH
        y_change = (Y_MAX - Y_MIN) / OG_WIDTH
        base = [-X_MAX * 2, Y_MAX - Y_MIN]
        origin = OG.info.origin.position
        current = [origin.x - self.smallbot.baseBotPose.position.x, origin.y - self.smallbot.baseBotPose.position.y]
        angle = self.angle_between_vectors(base, current)
        x = index % OG_WIDTH
        y = index / OG_WIDTH
        point = Pose()
        point.position.x = x * (math.cos(angle) * x_change + math.sin(angle) * y_change) + origin.x
        point.position.y = y * (math.cos(angle) * y_change + math.sin(angle) * x_change) + origin.y
        print("origin: " + str(origin))
        print("angle: " + str(angle))
        print("Converted Point: " + str(point))
        return point

    # converts a Pose.position to an index
    def point_to_index(self, point, OG):
        x_change = (X_MAX * 2) / OG_WIDTH
        y_change = (Y_MAX - Y_MIN) / OG_WIDTH
        base = [-X_MAX * 2, Y_MAX - Y_MIN]
        origin = OG.info.origin.position
        current = [origin.x - self.smallbot.baseBotPose.position.x, origin.y - self.smallbot.baseBotPose.position.y]
        angle = self.angle_between_vectors(base, current)
        x_index = (point.x - origin.x) / (x_change * math.cos(angle) + y_change * math.sin(angle))
        y_index = (point.y - origin.y) / (y_change * math.cos(angle) + x_change * math.sin(angle))
        return int(x_index + y_index * OG_WIDTH)

    # returns the angle between two vectors
    def angle_between_vectors(self, v1, v2):
        dot = float(v1[0]) * float(v2[0]) + float(v1[1]) * float(v2[1])
        lv1 = math.sqrt(float(math.pow(v1[0], 2) + math.pow(v1[1], 2)))
        lv2 = math.sqrt(float(math.pow(v2[0], 2) + math.pow(v2[1], 2)))
        x = dot / (lv1 * lv2)
        return math.acos(x)