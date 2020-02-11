#!/usr/bin/python
# title           :Filter.py
# description     :python moving average and median filter
# author          :Sean Tidd
# date            :2020-02-11
# version         :0.1
# notes           :
# python_version  :3.5
# ==============================================================================
import copy


class Filter:
    def __init__(self, size):
        self.size = size
        self.list = [0.0] * size


    def add_value(self, value):
        """
        Adds a value to the list of data
        :param value: newest data to be added
        :return:
        """
        for i in range(self.size-1,0,-1):
            self.list[i] = self.list[i-1]
        self.list[0] = value


    def clear_list(self):
        """
        Resets the list to empty
        :return:
        """
        self.list = [0] * self.size


    def get_average(self):
        """
        Gets the average of the list
        :return: float of the average
        """
        sum = 0.0
        for val in self.list:
            sum += val
        return float(sum/self.size)


    def get_median(self):
        """
        Gets the median of the list
        :return: the value at the middle index
        """
        temp_list = copy.deepcopy(self.list)
        temp_list.sort()
        index = int(self.size/2)
        return temp_list[index]

