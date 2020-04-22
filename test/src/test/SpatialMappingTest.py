#!/usr/bin/env python3
from support.Constants import *
from baseBot.MapMaker import MapMaker
from baseBot.MapManager import MapManager
from data.Zone import Zone
from baseBot.MeshAnalyzer import MeshAnalyzer
import matplotlib.pyplot as plt


def basic_test():
    mm = MapMaker()
    counter = 0
    ax = plt.subplot(1, 1, 1)
    frame_count = 0
    while frame_count < 100:
        mm.get_frame()
        mm.update_pose()
        counter += 1
        if counter > 1000:
            print("translation: " + str(mm.translation))
            print("orientation: " + str(mm.orientation))
            euler = mm.quaternion_to_euler(mm.orientation[0], mm.orientation[1], mm.orientation[2], mm.orientation[3])
            ax.plot(euler[0], euler[1], "or")
            counter = 0
            frame_count += 1
    plt.show()


def test_visible_corners():
    mm = MapManager()
    counter = 0
    ax = plt.subplot(1, 1, 1)
    frame_count = 0
    while frame_count < 100:
        mm.mapMaker.get_frame()
        mm.mapMaker.update_pose()
        counter += 1
        if counter > 1000:
            t = mm.mapMaker.translation
            q = mm.mapMaker.orientation
            print("translation: " + str(t))
            print("orientation: " + str(q))
            [tl, tr, br, bl] = mm.get_visible_area_corners(t, q)
            ax.plot(tl[0], tl[1], "or")
            ax.plot(tr[0], tr[1], "ob")
            ax.plot(br[0], br[1], "oy")
            ax.plot(bl[0], bl[1], "og")
            counter = 0
            frame_count += 1
    plt.show()


def test_visible_zones():
    mm = MapManager()
    ma = MeshAnalyzer(None)
    counter = 0
    while True:
        mm.mapMaker.get_frame()
        mm.mapMaker.update_pose()
        counter += 1
        if counter > 1000:
            t = mm.mapMaker.translation
            q = mm.mapMaker.orientation
            visible_corners = mm.get_visible_area_corners(t, q)
            visible_zone = Zone(visible_corners, -1)
            visible_zones = []
            for zone_index in range(0, 100):
                zone_corners = mm.get_corners_from_center(mm.get_center_from_zone_index(zone_index))
                visible = True
                for corner in zone_corners:
                    if not ma.is_point_in_zone(visible_zone, corner):
                        visible = False
                        break
                if visible:
                    visible_zones.append(zone_index)
            print("Visible Zones: " + str(visible_zones))


if __name__ == "__main__":
    test_visible_zones()