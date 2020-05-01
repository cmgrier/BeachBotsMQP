#!/usr/bin/env python
from baseBot.MapMaker import MapMaker
from baseBot.MeshAnalyzer import MeshAnalyzer
from geometry_msgs.msg import Pose
import math

# This class holds the current map and passes it between the other managers
# Also this class works with the mesh from the MapMaker class and converts it into useful information, such as an OG
from data.Zone import Zone
from support.Constants import *


class MapManager:

    def __init__(self):
        self.zones = list()
        self.mapMaker = MapMaker()
        self.landing_strip = []  # [top_left, top_right, bottom_right, bottom_left]
        self.map = self.mapMaker.get_map()  # is a map of difficulty of terrain
        self.occupancy_grid = []
        self.cleanedZones = []

    # method to update the
    def update(self):
        self.update_OG()

    # returns center of a given zone index as [x, y]
    def get_center_from_zone_index(self, zone):
        s = 1
        w = 2
        while zone >= math.pow(w, 2):
            w += 2
            s += 1

        total = w * 2 + (w - 2) * 2
        max = math.pow(w, 2) - 1
        min = max - total + 1

        x = 1
        y = s
        i = 0
        while i + min != zone:
            i += 1
            if i < w / 2:
                x += 1
            elif w / 2 <= i < w - 1 + w / 2:
                y -= 1
            elif w - 1 + w / 2 <= i < w - 1 + w - 1 + w / 2:
                x -= 1
            elif w - 1 + w - 1 + w / 2 <= i < w - 1 + w - 1 + w - 1 + w / 2:
                y += 1
            else:
                x += 1

        rx = (x - 1) * ZONE_WIDTH + ZONE_WIDTH / 2
        ry = (y - 1) * ZONE_LENGTH + ZONE_LENGTH / 2
        return [rx, ry]

    # returns the zone index that contains the given point. point must be in [x, y] format
    def get_zone_index_from_point(self, point):
        if point[0] < 0:
            xf = -1
        else:
            xf = 1

        if point[1] < 0:
            yf = -1
        else:
            yf = 1

        x = math.ceil(math.fabs(point[0]) / ZONE_WIDTH)
        y = math.ceil(math.fabs(point[1]) / ZONE_LENGTH)
        x = x * xf
        y = y * yf

        if math.fabs(x) > math.fabs(y):
            s = math.fabs(x)
        else:
            s = math.fabs(y)
        w = s * 2
        total = w * 2 + (w - 2) * 2
        max = math.pow(w, 2) - 1
        min = max - total + 1

        if x == 0 and y == 0:
            zone = 0.0

        elif x == 0:
            zone = min
            if y < 0:
                zone += total / 2

        elif y == 0:
            zone = min + w / 2 - 1
            if x < 0:
                zone += total / 2

        elif x > 0 and y == w/2:
            zone = min - 1
            zone += x

        elif x == w/2 and y > 0:
            zone = min - 1 + w
            zone -= y

        elif x == w/2 and y < 0:
            zone = min - 1 + w - 1
            zone += math.fabs(y)

        elif y == -w/2 and x > 0:
            zone = min - 1 + 3 * w / 2 - 1
            zone += w/2 - x

        elif y == -w/2 and x < 0:
            zone = min - 1 + w / 2 + w - 1 + w / 2 - 1
            zone += math.fabs(x)

        elif x == - w/2 and y < 0:
            zone = min - 1 + w / 2 + w - 1 + w - 1
            zone += w/2 + y

        elif x == -w/2 and y > 0:
            zone = max - w + 1
            zone += y

        elif y == w/2 and x < 0:
            zone = max + 1
            zone += x

        else:
            zone = -1
            print("Error finding zone, no cases match")

        return int(zone)

    # returns the corners of a zone given a zone center in [x, y] format
    def get_corners_from_center(self, center):
        tl = [center[0] - ZONE_WIDTH / 2, center[1] + ZONE_LENGTH / 2]
        tr = [center[0] + ZONE_WIDTH / 2, center[1] + ZONE_LENGTH / 2]
        br = [center[0] + ZONE_WIDTH / 2, center[1] - ZONE_LENGTH / 2]
        bl = [center[0] - ZONE_WIDTH / 2, center[1] - ZONE_LENGTH / 2]
        return [tl, tr, br, bl]

    # returns the visible zones from given position (t) and orientation (o)
    def get_visible_zones(self, t, o):
        if DEBUG:
            print("Get visible zones...")
        visible_area_corners = self.get_visible_area_corners(t, o)
        visible_area_zone = Zone(visible_area_corners, -1)
        if DEBUG:
            print("visible area corners: " + str(visible_area_corners))

        mesh = self.mapMaker.mesh
        ma = MeshAnalyzer(mesh)
        possible_zones = set()
        if DEBUG:
            print("num vertices: " + str(len(mesh.vertices)))
        for vertex in mesh.vertices:
            possible_zones.add(self.get_zone_index_from_point(vertex))

        if DEBUG:
            print("possible zones: " + str(possible_zones))
        visible_zones = []
        for zone in possible_zones:
            corners = self.get_corners_from_center(self.get_center_from_zone_index(zone))
            visible = True
            for corner in corners:
                if not ma.is_point_in_zone(visible_area_zone, corner):
                    visible = False
                    break
            if visible:
                visible_zones.append(zone)
        if DEBUG:
            print("num visible zones: " + str(len(visible_zones)))
        return visible_zones

    # updates self.zones with all visible zones that have not been cleaned yet. This should be called in main loop
    def update_zones(self):
        visible_zones = self.get_visible_zones(self.mapMaker.translation, self.mapMaker.orientation)
        zones_to_add = []
        for zone in visible_zones:
            should_add = True
            for cleaned_zone in self.cleanedZones:
                if cleaned_zone.id == zone:
                    should_add = False
                    break

            for identified_zone in self.zones:
                if identified_zone.id == zone:
                    should_add = False

            if should_add:
                zones_to_add.append(zone)

        for zone_index in zones_to_add:
            corners = self.get_corners_from_center(self.get_center_from_zone_index(zone_index))
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
            aZone = Zone([tl, tr, br, bl], zone_index)
            self.zones.append(aZone)

    # returns an [x, y] rotated by a given angle (radian) around a given center point [x, y]
    def rotate_point_around_point(self, center, point, angle):
        rx = math.cos(angle) * (point[0] - center[0]) - math.sin(angle) * (point[1] - center[1]) + center[0]
        ry = math.sin(angle) * (point[0] - center[0]) + math.cos(angle) * (point[1] - center[1]) + center[1]
        return [rx, ry]

    # converts given quaternion [x, y, z, w] to radian euler angles
    def quaternion_to_euler(self, q):
        x = q[0]
        y = q[1]
        z = q[2]
        w = q[3]

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]

    # returns the corners of the visible area in front of the ZED
    def get_visible_area_corners(self, t, o):
        tl = [t[0] - X_MAX, t[1] + Y_MAX]
        tr = [t[0] + X_MAX, t[1] + Y_MAX]
        br = [t[0] + X_MAX, t[1] + Y_MIN]
        bl = [t[0] - X_MAX, t[1] + Y_MIN]

        euler = self.quaternion_to_euler(o)

        tl = self.rotate_point_around_point(t, tl, euler[2])
        tr = self.rotate_point_around_point(t, tr, euler[2])
        br = self.rotate_point_around_point(t, br, euler[2])
        bl = self.rotate_point_around_point(t, bl, euler[2])
        return [tl, tr, br, bl]

    # makes an OG of area in front of ZED and updates self.occupancy_grid with it
    def update_OG(self):
        mesh = self.mapMaker.mesh
        ma = MeshAnalyzer(mesh)
        corners = self.get_visible_area_corners(self.mapMaker.translation, self.mapMaker.orientation)
        visible_zones = self.get_visible_zones(self.mapMaker.translation, self.mapMaker.orientation)
        full_zones = []
        for zone_id in visible_zones:
            center = self.get_center_from_zone_index(zone_id)
            corners = self.get_corners_from_center(center)
            z = Zone(corners, zone_id)
            full_zones.append(z)
        OG = ma.make_occupancy_grid_bl(full_zones, corners)
        self.occupancy_grid = OG

    """
    The Following is Legacy and should be ignored
    """

    # sets the current map from mapMaker and creates zones
    def update_map(self):
        self.map = self.mapMaker.get_map()
        self.divide_map()

    def divide_map(self):
        self.__create_zones()
        self.__create_landing_strip()

    # divides the map into different zones
    def __create_zones(self):
        self.zones = list()
        map_width = int(math.sqrt(len(self.map)))
        number_of_zones = int(map_width / ZONE_WIDTH)

        for i in range(0, number_of_zones - 1):
            top_left = i * ZONE_WIDTH
            top_right = top_left + ZONE_WIDTH
            bottom_left = top_left + map_width * (map_width - LANDING_STRIP_WIDTH)
            bottom_right = bottom_left + ZONE_WIDTH
            zone_corners = [top_left, top_right, bottom_right, bottom_left]
            new_zone = Zone(zone_corners, i)
            self.zones.append(new_zone)

        top_left = self.zones[len(self.zones) - 1].corners[1]
        top_right = map_width - 1
        bottom_left = self.zones[len(self.zones) - 1].corners[3]
        bottom_right = top_right + map_width * (map_width - LANDING_STRIP_WIDTH)
        new_zone = Zone([top_left, top_right, bottom_right, bottom_left], len(self.zones))
        self.zones.append(new_zone)

    def __create_landing_strip(self):
        map_width = int(math.sqrt(len(self.map)))
        top_left = map_width * (map_width - LANDING_STRIP_WIDTH)
        top_right = top_left + map_width
        bottom_right = len(self.map) - 1
        bottom_left = bottom_right - map_width
        self.landing_strip = [top_left, top_right, bottom_right, bottom_left]

    def percent_of_indexes_safe(self, index_list):
        indexes_too_difficult = 0.0
        for index in index_list:
            if self.map[index] > TERRAIN_TOO_DIFFICULT or self.map[index] == -1:
                indexes_too_difficult += 1.0

        return indexes_too_difficult / float(len(index_list))

    def point_to_index(self, position):
        return 1
