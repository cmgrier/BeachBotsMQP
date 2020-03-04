from support.EqualPriorityQueue import EqualPriorityQueue
from small_bot.TaskManager import TaskManager
from small_bot.TaskSeeker import TaskSeeker
from data.Task import Task
from geometry_msgs.msg import Pose, Twist, PointStamped, PolygonStamped, Point32
from small_bot.msg import AvoidAlert
from nav_msgs.msg import OccupancyGrid
import rospy


class SmallBotManager:
    def __init__(self):
        self.isCleaning = True
        self.tasks = EqualPriorityQueue()
        self.id = -1
        self.position = Pose()
        self.baseBotPose = Pose()
        self.occupancyGrid = OccupancyGrid()
        self.taskManager = TaskManager(self)
        self.taskSeeker = TaskSeeker(self)
        self.request_id()
        topic_name = 'robo_pos' + str(self.id)
        self.position_publisher = rospy.Publisher(topic_name, PointStamped, queue_size=10)
        self.zone_publisher = rospy.Publisher('robo' + str(self.id) + 'zone', PolygonStamped, queue_size=10)

        if self.id != -1:
            self.main()

    def main(self):
        """
        Main loop for small robot
        :return:
        """
        while self.isCleaning:
            rospy.sleep(.01)
            self.get_info()
            if len(self.tasks) > 0:
                task = self.tasks.get()
                if task.type == "end":
                    break
                updated_task = self.do_task(task)
                if not updated_task.isComplete:
                    self.tasks.put(updated_task.priority, updated_task)
                else:
                    print("task " + str(task.zone.id) + " is COMPLETE")
                    twist = Twist()
                    self.taskManager.pub_vel(twist)

    def get_info(self):
        """
        Gathers info from any ros services or other communication
        :return:
        """
        # check the services and update SmallBotManager's info
        clean = 3
        if self.tasks.has(clean) == False:
            self.taskSeeker.request_clean_task()
        self.taskSeeker.request_avoid_status()
        self.taskSeeker.request_og()
        #TODO update robots position as well as add more info
        #Maybe have a ros listener for current position
        #If dump satus is true then request dump task

    def do_task(self, task):
        """
        Sends a task to be identified and executed
        :param task: The task to be executed
        :return: An updated Task based on where it was left off
        """
        return self.taskManager.do_task(task)

    def request_id(self):
        """
        Requests basebot to ID smallbot
        :return:
        """
        self.taskSeeker.request_ID()
        topic = "avoid_alert_" + str(self.id)
        rospy.Subscriber(topic, AvoidAlert, self.avoidListener)
        pass

    def avoidListener(self, data):
        if data.type == "avoid":
            task = self.taskSeeker.parse_task(data)
            self.tasks.put(task.priority, task)

    def publish_pos(self, pos=None):
        ps = PointStamped()
        ps.header.frame_id = "/map"
        if pos is not None:
            ps.point = pos.position
        else:
            ps.point = self.position.position
        self.position_publisher.publish(ps)

    def publish_zone_shape(self, task):
        zone = PolygonStamped()
        zone.header.frame_id = "/map"
        points = []
        for corner in task.zone.corners:
            p = Point32()
            p.x = corner.position.x
            p.y = corner.position.y
            p.z = corner.position.z
            points.append(p)
        zone.polygon.points = points
        self.zone_publisher.publish(zone)


if __name__ == "__main__":
    smallbot = SmallBotManager()
