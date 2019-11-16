"""
This class is similar to the default PriorityQueue class except it handles the case of two entities with equal priority
by adding the new one and removing the other entity found in the queue
"""

class EqualPriorityQueue:
    def __init__(self):
        self.queue = []


    def put(self, priority, data):
        """
        Adds an entity to the priority queue and replaces any old entities with equal priority
        :param priority: priority level of the data type from ascending order
        :param data: data to be added to the queue
        :return: void
        """
        self.remove(priority)
        self.queue.append((priority, data))
        self.queue = sorted(self.queue, key=lambda x: float(x[0]))


    def get(self):
        """
        Pops the highest priority item amd returns only the data
        :return: data from the queue
        """
        item = self.queue.pop(0)
        return item[1]

    def remove(self, priority):
        """
        Removes any entity from the queue with the given priority level
        :param priority: priority level of the entity to remove
        :return: void
        """
        for stuff in self.queue:
            if priority is stuff[0]:
                self.queue.remove(stuff)
                break

    def size(self):
        """
        Gets the size of the queue
        :return: size of queue
        """
        return len(self.queue)


    def empty(self):
        """
        Identifies if the queue is empty or not
        :return: a boolean for true if empty and false if otherwise
        """
        if self.size() is 0:
            return True
        else:
            return False


    def has(self, priority):
        """
        Checks to see if an item of the specified priority is in the queue
        :param priority: The item's priority to search
        :return: True if item was found
        """
        for item in self.queue:
            if item[0] == priority:
                return True
        return False


    def __getitem__(self, item):
        """
        Magic function to allow printing parts of this list from the class
        :param item: index to print from the queue
        :return: a string of the values to print
        """
        return str(self.queue[item])


    def __str__(self):
        """
        Magic method to allow printing of whole list from the class
        :return: a string of values to print
        """
        return str(self.queue)

    def __repr__(self):
        return str(self.queue)

    def __len__(self):
        return len(self.queue)

if __name__ == "__main__":
    queue = EqualPriorityQueue()
    queue.put(1, 24)
    queue.put(3, "Truth Hurts")
    queue.put(2, "I Love You")

    print(queue[2])
