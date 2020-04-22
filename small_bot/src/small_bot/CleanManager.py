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
        self.testing = False
        self.simulating = True
        self.OG_real = True
        self.sim_pos = Pose()
        pass

    def do_task(self, task):
        # this will attempt to complete the given clean task,
        # update progress on task and return the updated task
        if DEBUG:
            print("Waypoints left: " + str(len(self.waypoints)))
        self.smallbot.publish_zone_shape(task)
        if self.testing:
            return self.do_task_test(task)
        if self.current_task_id != task.zone.id:
            self.current_task_id = task.zone.id
            self.waypoints = self.make_path_in_zone(task)
            print("added new waypoints for task " + str(task.zone.id))
        elif len(self.waypoints) > 0:
            self.counter += 1
            if self.simulating and not self.OG_real:
                if self.is_at_position(self.sim_pos, self.waypoints[0]):
                    self.waypoints.pop(0)
                    print("Waypoint Reached!")
                else:
                    self.increment_sim_pos(self.waypoints[0])
                    if DEBUG:
                        print("SIM POS: ")
                        print(self.sim_pos)
                    self.smallbot.publish_pos(self.sim_pos)
            elif self.simulating and self.OG_real:
                self.counter += 1
                if self.is_at_position(self.sim_pos, self.waypoints[0]):
                    self.waypoints.pop(0)
                    print("Waypoint Reached!")
                else:
                    self.nav_to_point(self.waypoints[0], self.smallbot.occupancyGrid)
                    if DEBUG:
                        print("SIM POS: ")
                        print(self.sim_pos)
            else:
                if self.is_at_position(self.smallbot.position, self.waypoints[0]):
                    self.waypoints.pop(0)
                else:
                    self.nav_to_point(self.waypoints[0], self.smallbot.occupancyGrid)
        elif len(self.waypoints) == 0:
            print("Cleaning Task " + str(task.zone.id) + " COMPLETE")
            task.isComplete = True
        return task

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

    def increment_sim_pos(self, goal):
        change = .05
        vector = [goal.position.x - self.sim_pos.position.x, goal.position.y - self.sim_pos.position.y]
        mag = math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1], 2))
        norm = [vector[0] / mag, vector[1] / mag]
        new_pos = Pose()
        new_pos.position.x = self.sim_pos.position.x + norm[0] * change
        new_pos.position.y = self.sim_pos.position.y + norm[1] * change
        self.sim_pos = new_pos

    # returns True if this robot is at the given pose
    def is_at_position(self, current_pose, goal_pose):
        x = current_pose.position.x
        y = current_pose.position.y
        return goal_pose.position.x - POSITION_THRESHOLD < x < goal_pose.position.x + POSITION_THRESHOLD \
            and goal_pose.position.y - POSITION_THRESHOLD < y < goal_pose.position.y + POSITION_THRESHOLD

    # astar to the given point from current position
    def nav_to_point(self, pose, OG):
        astar = AStar(OG.data)
        start = self.point_to_index(self.smallbot.position.position, OG)
        goal = self.point_to_index(pose.position, OG)
        path = astar.find_path(start, goal)
        points = self.get_path_of_points(path, OG)
        if self.simulating:
            if self.is_at_position(self.sim_pos, points[0]):
                points.pop(0)
            self.increment_sim_pos(points[0])
            self.smallbot.publish_pos(self.sim_pos)

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
            point.position.y = y_pos + y_increment
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
        if DEBUG:
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
        if DEBUG:
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
        index = int(x_index + y_index * OG_WIDTH)
        if index < 0:

        return

    # returns the angle between two vectors
    def angle_between_vectors(self, v1, v2):
        dot = float(v1[0]) * float(v2[0]) + float(v1[1]) * float(v2[1])
        lv1 = math.sqrt(float(math.pow(v1[0], 2) + math.pow(v1[1], 2)))
        lv2 = math.sqrt(float(math.pow(v2[0], 2) + math.pow(v2[1], 2)))
        x = dot / (lv1 * lv2)
        return math.acos(x)