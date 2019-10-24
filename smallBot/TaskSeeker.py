from smallBot.TaskIdentifier import TaskIdentifier
#from smallBot.TaskStatus import TaskStatus
from data.Task import Task
from queue import PriorityQueue

#This class resquest a new task and sends that task off to be identified.
#This class also updates the status for the avoid and return tasks
class TaskSeeker:

    def __init__(self):
        self.TaskIdentifier = TaskIdentifier()
        self.currentTask = None
        self.Tasks = PriorityQueue()
        self.request()
        #self.send_to_id() TODO delete commemt




    #Requests a new task from the base bot and updates the currentTask variable
    def request(self):
        task1 = Task(3)
        task2 = Task(2, "avoid")
        task3 = Task(4, "dump")
        task4 = Task(4, "clean")
        self.Tasks.put((task1.priority, task1))
        self.Tasks.put((task2.priority, task2))
        self.Tasks.put((task3.priority, task3))
        return #TODO make the service request for the Task



    #When service is requested by the basebot, updates both the avoid status to true and the coordinate
    def update_avoid_status(self):
        return
    #TODO make the service to update the avoid status and avoid coordinate from Task



    #When service is requested, updates amd adds a dump task to the priority queue
    def update_dump_status(self):
        return



    #Send task to be identified for appropriate execution
    def send_to_id(self):
        self.currentTask = self.Tasks.get()[1]
        self.TaskIdentifier.task_ID(self.currentTask)



if __name__ == "__main__":
    ts = TaskSeeker()
    while not ts.Tasks.empty():
        ts.send_to_id()