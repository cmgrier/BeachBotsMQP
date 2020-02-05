#!/usr/bin/env python3
import math
import pyzed.sl as sl
import time
import random

from data.Zone import Zone
from baseBot.MapMaker import MapMaker
from baseBot.MeshAnalyzer import MeshAnalyzer
from baseBot.MapManager import MapManager
from support.Constants import *

def full_test():
    vertices = []
    for i in range(100):
        vertex = [0, 0, 0]
        vertex[0] = random.randint(-3, -3)
        vertex[1] = random.randint(0, 3)
        vertex[2] = random.randint(-1, 3)
        vertices.append(vertex)

    mapManager = MapManager()
    timer = 0

    print("Getting Frames...")
    # Grab 500 frames and stop
    while timer < 500:
        mapManager.mapMaker.update_map_async()
        if mapManager.mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1

    ma = MeshAnalyzer(mapManager.mapMaker.mesh)

    print("starting test")
    area_corners = mapManager.get_visible_area_corners(mapManager.mapMaker.translation, mapManager.mapMaker.orientation)
    visible_zones = mapManager.get_visible_zones(mapManager.mapMaker.translation, mapManager.mapMaker.orientation)
    full_zones = []
    for zone_id in visible_zones:
        center = mapManager.get_center_from_zone_index(zone_id)
        corners = mapManager.get_corners_from_center(center)
        z = Zone(corners, zone_id)
        full_zones.append(z)
    OG = ma.make_occupancy_grid_in_front(full_zones, area_corners)

    print(OG)


def simple_test():
    mesh = sl.Mesh()

    ma = MeshAnalyzer(mesh)

    p0 = [0, 0]
    p1 = [0, 1]
    p2 = [0, 2]
    p3 = [1, 0]
    p4 = [1, 1]
    p5 = [1, 2]
    p6 = [2, 0]
    p7 = [2, 1]
    p8 = [2, 2]
    p9 = [0, 0.001]

    """
    print(ma.area_of_triangle(p1, p0, p6))
    print(ma.area_of_triangle(p1, p0, p8))
    print(ma.area_of_triangle(p1, p8, p6))
    print(ma.area_of_triangle(p8, p0, p6))
    """

    # check edges and corners
    assert ma.is_point_in_triangle_simple(p4, p0, p6, p8) is True
    assert ma.is_point_in_triangle_simple(p3, p0, p6, p8) is True
    assert ma.is_point_in_triangle_simple(p7, p0, p6, p8) is True
    assert ma.is_point_in_triangle_simple(p0, p0, p6, p8) is True
    assert ma.is_point_in_triangle_simple(p6, p0, p6, p8) is True
    assert ma.is_point_in_triangle_simple(p8, p0, p6, p8) is True

    # check outside
    assert ma.is_point_in_triangle_simple(p1, p0, p6, p8) is False
    assert ma.is_point_in_triangle_simple(p3, p0, p2, p8) is False
    assert ma.is_point_in_triangle_simple(p5, p0, p2, p6) is False
    assert ma.is_point_in_triangle_simple(p3, p2, p6, p8) is False

    # check close
    assert ma.is_point_in_triangle_simple(p9, p0, p6, p8) is False


if __name__ == '__main__':
    full_test()
