#!/usr/bin/env python
import math
from _heapq import *

from support.Constants import *


class AStar:

    # given map is an array of weights
    def __init__(self, map):
        self.__map = map
        self.__frontier = list()
        self.__investigated = list()

    # start and end are indexes on the map
    # map indexes work as follows:
    #    x ->
    # y  0  1  2  3
    # |  4  5  6  7
    # v  8  9  10 11
    #    12 13 14 15
    #       \  /
    #       robot

    # method to be called outside of AStar class to retrieve path
    def find_path(self, start, end):
        if start == end:
            return [start]
        self.__frontier = list()
        node = Node()
        node.index = start
        node.heuristic = self.__get_heuristic(start, end)
        node.is_start = True
        heappush(self.__frontier, [node, node.cost])

        while self.__frontier.__len__() > 0:
            best_node = heappop(self.__frontier)[0]
            if best_node.index == end:
                return self.__create_path(start, best_node)
            self.__handle_new_nodes(self.__investigate_neighbors(best_node, end))
            self.__investigated.append(best_node)
        return start

    # checks local points above, below, and to either side of given node
    # and returns a list of possible nodes
    def __investigate_neighbors(self, node, end):
        new_nodes = list()
        if node.index - 1 >= 0 \
                and node.index % self.__map.width != 0 \
                and self.__map.map[node.index - 1] != -1:
            left_node = self.__create_node(node.index - 1,
                                           end,
                                           node.movement_cost + STANDARD_MOVE_COST,
                                           node)
            new_nodes.append(left_node)

        if node.index + 1 < self.__map.width * self.__map.width \
                and node.index % self.__map.width != 3 \
                and self.__map.map[node.index + 1] != -1:
            right_node = self.__create_node(node.index + 1,
                                            end,
                                            node.movement_cost + STANDARD_MOVE_COST,
                                            node)
            new_nodes.append(right_node)

        if node.index - self.__map.width > 0 \
                and self.__map.map[node.index - self.__map.width] != -1:
            top_node = self.__create_node(node.index - self.__map.width,
                                          end,
                                          node.movement_cost + STANDARD_MOVE_COST,
                                          node)
            new_nodes.append(top_node)

        if node.index + self.__map.width < self.__map.width * self.__map.width \
                and self.__map.map[node.index + self.__map.width] != -1:
            bottom_node = self.__create_node(node.index + self.__map.width,
                                             end,
                                             node.movement_cost + STANDARD_MOVE_COST,
                                             node)
            new_nodes.append(bottom_node)

        return new_nodes

    # determines whether to update an existing node or to add the new node to the frontier
    def __handle_new_nodes(self, new_nodes):
        for new_node in new_nodes:
            if self.__is_node_in_frontier(new_node) and not self.__is_node_investigated(new_node):
                self.__update_node(new_node)
            elif not self.__is_node_investigated(new_node):
                heappush(self.__frontier, [new_node, new_node.cost])

    # checks if the node is already in frontier
    def __is_node_in_frontier(self, node):
        for frontier_node in self.__frontier:
            if frontier_node.index == node.index:
                return True
        return False

    # checks if the node has already been investigated
    def __is_node_investigated(self, node):
        for investigated_node in self.__investigated:
            if investigated_node.index == node.index:
                return True
        return False

    # Updates the node if it already exists in the frontier and has a shorter path
    def __update_node(self, new_node):
        for node in self.__frontier:
            self.__update_node_cost(new_node, node)

    def __get_direction(self, node):
        # 1 is horizontal direction, 0 is vertical
        if abs(node.index - node.previous_point.index) > 1:
            return 1
        else:
            return 0

    def __is_turning(self, node):
        return self.__get_direction(node) != self.__get_direction(node.previous_point)

    # helper for self.update_node_in_frontier
    def __update_node_cost(self, new_node, old_node):
        if old_node.index == new_node.index and new_node.cost < old_node.cost:
            self.__frontier.remove([old_node, old_node.cost])
            heapify(self.__frontier)
            heappush(self.__frontier, [new_node, new_node.cost])

    # this creates path from the given start and end node
    def __create_path(self, start_index, end):
        path = list()
        path.append(end.index)
        current_node = end.previous_point
        while current_node.index != start_index:
            path.append(current_node.index)
            current_node = current_node.previous_point
        path.append(start_index)
        path.reverse()
        return path

    # creates a new node object from the given index, end index, and current movement cost
    def __create_node(self, position, end, movement_cost, origin):
        node = Node()
        node.heuristic = self.__get_heuristic(position, end)
        node.movement_cost = movement_cost + self.__map.map[position] * DIFFICULT_TERRAIN_FACTOR
        node.index = position
        node.cost = node.heuristic + node.movement_cost
        node.previous_point = origin
        if not node.previous_point.is_start and self.__is_turning(node):
            node.movement_cost += TURNING_FACTOR
        return node

    # returns the estimated cost between the two given points
    def __get_heuristic(self, start, end):
        start_point = self.__convert_index_to_point(start)
        end_point = self.__convert_index_to_point(end)
        return math.sqrt(math.pow(start_point[0] - end_point[0], 2) + math.pow(start_point[1] - end_point[1], 2))

    # converts the given index to an xy point. Used to find heuristic
    def __convert_index_to_point(self, index):
        x = int(index % self.__map.width)
        y = int(index / self.__map.width)
        point = [x, y]
        return point


# data classes used for AStar. Shouldn't be used anywhere else
class Node:
    cost = 0
    heuristic = 0
    movement_cost = 0
    index = 0
    previous_point = None
    is_start = False

    def __lt__(self, other):
        if isinstance(other, self.__class__):
            return self.cost < other.cost
        return NotImplemented


class Map:
    # occupancy grid where values != -1 are able to be traversed with 0 being easy to cross
    # -1 is unable to traverse
    map = list()
    width = 0


# for testing
if __name__ == '__main__':
    map = Map()
    map.map = [0, 0, 0, 0,
               -1, 0, 0, 0,
               0, 0, -1, 0,
               0, 0, 0, 0]  # length = 16
    map.width = int(math.sqrt(map.map.__len__()))
    a_star = AStar(map)
    path = a_star.find_path(0, 14)
    print(path)