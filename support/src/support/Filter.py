#!/usr/bin/python
import copy


class Filter:
    def __init__(self, size):
        self.size = size
        self.list = [0.0] * size


    def addValue(self, value):
        """
        Adds a value to the list of data
        :param value: newest data to be added
        :return:
        """
        for i in range(self.size-1,0,-1):
            self.list[i] = self.list[i-1]
        self.list[0] = value


    def clearList(self):
        """
        Resets the list to empty
        :return:
        """
        self.list = [0] * self.size


    def getAverage(self):
        """
        Gets the average of the list
        :return: float of the average
        """
        sum = 0.0
        for val in self.list:
            sum += val
        return float(sum/self.size)


    def getMedian(self):
        """
        Gets the median of the list
        :return: the value at the middle index
        """
        tempList = copy.deepcopy(self.list)
        tempList.sort()
        index = int(self.size/2)
        return tempList[index]

