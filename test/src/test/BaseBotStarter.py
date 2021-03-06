#!/usr/bin/env python3

import rospy
from baseBot.CleaningManager import CleaningManager
from geometry_msgs.msg import Pose
from data.Task import Task
from data.Zone import Zone
from support.Constants import *
import time
import pyzed.sl as sl

def zed_test():
    emptyRobotList = []
    taskID = 0
    CM = CleaningManager(emptyRobotList)
    print("Getting Frames...")
    timer = 0
    # Grab 500 frames and stop
    while timer < 500:
        CM.mapManager.mapMaker.update_map_async()
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1

    CM.mapManager.update_zones()
    CM.create_cleaning_tasks()
    print(len(CM.cleaningTasks))
    print(CM.cleaningTasks[0])
    print(CM.cleaningTasks[0].zone.corners)

def basic_test():
    emptyRobotList = []
    taskID = 0
    CM = CleaningManager(emptyRobotList)
    cleaningTask1 = Task(Zone([Pose(), Pose(), Pose(), Pose()], taskID))
    CM.cleaningTasks.append(cleaningTask1)
    while 1:
        if len(CM.cleaningTasks) < 1:
            taskID = taskID + 1
            cleaningTask = Task(Zone([Pose(), Pose(), Pose(), Pose()], taskID))
            CM.cleaningTasks.append(cleaningTask)
            print("added new cleaning task with ID:")
            print(taskID)
        time.sleep(.5)
        # rospy.sleep(.01)

def og_test():
    emptyRobotList = []
    CM = CleaningManager(emptyRobotList)
    timer = 0
    print("Getting Frames...")
    while timer < 500:
        CM.mapManager.mapMaker.update_map_async()
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1
    CM.mapManager.update_OG()
    while 1:
        CM.robotManager.director.publish_og()
        rospy.sleep(.01)

def continuous_og_test():
    emptyRobotList = []
    CM = CleaningManager(emptyRobotList)
    timer = 0
    print("Getting Frames...")
    while timer < 500:
        CM.mapManager.mapMaker.update_map_async()
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1
    CM.mapManager.update_OG()
    timer = 0
    while 1:
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            timer += 1
        if timer == 50:
            "updating OG..."
            CM.mapManager.mapMaker.update_pose_z()
            CM.mapManager.mapMaker.update_map_async()
            CM.mapManager.update_OG()
            CM.robotManager.director.publish_og()
            timer = 0

def basic_test_with_camera():
    emptyRobotList = []
    taskID = 0
    CM = CleaningManager(emptyRobotList)
    timer = 0
    print("Getting Frames...")
    while timer < 500:
        CM.mapManager.mapMaker.update_map_async()
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1
    CM.mapManager.update_OG()
    timer = 0
    cleaningTask1 = Task(Zone([Pose(), Pose(), Pose(), Pose()], taskID))
    CM.cleaningTasks.append(cleaningTask1)
    CM.mapManager.get_visible_zones()
    while 1:
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            timer += 1

        if timer == 50:
            "updating OG..."
            CM.mapManager.mapMaker.update_pose_z()
            CM.mapManager.mapMaker.update_map_async()
            CM.mapManager.update_OG()
            CM.robotManager.director.publish_og()
            timer = 0

        if len(CM.cleaningTasks) < 1:
            taskID = taskID + 1
            corners = CM.mapManager.get_corners_from_center(CM.mapManager.get_center_from_zone_index(taskID))
            tl = Pose()
            tl.position.x = corners[0][0]
            tl.position.y = corners[0][1]
            tr = Pose()
            tr.position.x = corners[1][0]
            tr.position.y = corners[1][1]
            br = Pose()
            br.position.x = corners[2][0]
            br.position.y = corners[2][1]
            bl = Pose()
            bl.position.x = corners[3][0]
            bl.position.y = corners[3][1]
            cleaningTask = Task(Zone([tl, tr, br, bl], taskID))
            #cleaningTask = Task(Zone([Pose(), Pose(), Pose(), Pose()], taskID))
            CM.cleaningTasks.append(cleaningTask)
            print("added new cleaning task with ID:")
            print(taskID)
        time.sleep(.5)


def advanced_test_with_camera():
    emptyRobotList = []
    CM = CleaningManager(emptyRobotList)
    timer = 0
    print("Getting Frames...")
    while timer < 500:
        CM.mapManager.mapMaker.update_map_async()
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1
    CM.mapManager.update_OG()
    timer = 0
    CM.mapManager.update_zones()
    CM.create_cleaning_tasks()
    while 1:
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            timer += 1

        if timer == 50:
            "updating OG..."
            CM.mapManager.mapMaker.update_pose()
            CM.mapManager.mapMaker.update_map_async()
            CM.mapManager.update_OG()
            CM.robotManager.director.publish_og()
            timer = 0

        time.sleep(.5)


def test_zone_identification_with_movement():
    emptyRobotList = []
    CM = CleaningManager(emptyRobotList)
    timer = 0
    print("Getting Frames...")
    while timer < 500:
        CM.mapManager.mapMaker.update_map_async()
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1
    timer = 0
    while True:
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            timer += 1
        if timer == 50:
            print("updating OG...")
            CM.mapManager.mapMaker.update_pose()
            CM.mapManager.mapMaker.update_map_async(True)
            CM.mapManager.update_OG()
            CM.robotManager.director.publish_og()
            CM.robotManager.director.publish_visible_area(CM.mapManager.mapMaker.translation, CM.mapManager.mapMaker.orientation)
            CM.robotManager.director.publish_all_visible_zones(CM.mapManager.mapMaker.translation, CM.mapManager.mapMaker.orientation)
            CM.robotManager.director.publish_position(CM.mapManager.mapMaker.translation, CM.mapManager.mapMaker.orientation)
            timer = 0
            print("done")
            zone_strs = ""
            for zone in CM.mapManager.zones:
                zone_strs += str(zone.id) + ", "
            print("current zones: " + str(zone_strs))
            print("visible area corners: " + str(CM.mapManager.get_visible_area_corners(CM.mapManager.mapMaker.translation, CM.mapManager.mapMaker.orientation)))
            CM.mapManager.update_zones()
            CM.create_cleaning_tasks()
            visible_zones = CM.mapManager.get_visible_zones(CM.mapManager.mapMaker.translation, CM.mapManager.mapMaker.orientation)
            #print("Visible Zones: " + str(visible_zones))
            #print("new zones: " + str(CM.mapManager.zones))


def final_test_full():
    emptyRobotList = []
    CM = CleaningManager(emptyRobotList)
    timer = 0
    print("Getting Frames...")
    while timer < 500:
        CM.mapManager.mapMaker.update_map_async()
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1
    timer = 0
    print("Ready")
    while True:
        if CM.mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            timer += 1
        if timer == 50:
            if DEBUG:
                print("updating OG...")
            CM.mapManager.mapMaker.update_pose()
            CM.mapManager.mapMaker.update_map_async(True)
            CM.mapManager.update_OG()
            CM.robotManager.director.publish_og()
            CM.robotManager.director.publish_visible_area(CM.mapManager.mapMaker.translation,
                                                          CM.mapManager.mapMaker.orientation)
            CM.robotManager.director.publish_all_visible_zones(CM.mapManager.mapMaker.translation,
                                                               CM.mapManager.mapMaker.orientation)
            CM.robotManager.director.publish_position(CM.mapManager.mapMaker.translation,
                                                      CM.mapManager.mapMaker.orientation)
            timer = 0
            if DEBUG:
                print("done")
                zone_strs = ""
                for zone in CM.mapManager.zones:
                    zone_strs += str(zone.id) + ", "
                print("current zones: " + str(zone_strs))
                print("visible area corners: {0}".format(str(
                    CM.mapManager.get_visible_area_corners(CM.mapManager.mapMaker.translation,
                                                           CM.mapManager.mapMaker.orientation))))
            CM.mapManager.update_zones()
            CM.create_cleaning_tasks()


if __name__ == "__main__":
    print("Starting...")
    final_test_full()