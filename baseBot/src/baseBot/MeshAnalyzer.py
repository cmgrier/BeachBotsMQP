#!/usr/bin/env python3
import pyzed.sl as sl
import time

import rospy
import math

from support.Constants import *

# creates maps for the Map Manager class
class MeshAnalyzer:
    def __init__(self, mesh):
        self.mesh = mesh

    """
    VERTICES
    """

    def get_lowest_vertices(self, vertices, range):
        lowest = self.get_lowest_vertex(vertices)
        lowest_vertices = []
        for vertex in vertices:
            if vertex[2] < lowest[2] + range:
                lowest_vertices.append(vertex)
        return lowest_vertices

    def get_lowest_vertex(self, vertices):
        lowest = vertices[0]
        for vertex in vertices:
            if vertex[2] < lowest[2]:
                lowest = vertex
        return lowest

    def sort_by_height(self, vertices):
        sorted(vertices, key=lambda x: x[2])
        return vertices

    def get_avg_height(self, vertices):
        length = len(vertices)
        height_sum = 0
        for vertex in vertices:
            height_sum += vertex[2]
        return height_sum / length

    def make_vector(self, vertex1, vertex2):
        vector = [0, 0, 0]
        vector[0] = vertex2[0] - vertex1[0]
        vector[1] = vertex2[1] - vertex1[1]
        vector[2] = vertex2[2] - vertex1[2]
        return vector

    def center_vertices(self, vertices, radius):
        center_vertices = []
        y_bounds = self.get_bounds(vertices, 1)
        x_bounds = self.get_bounds(vertices, 0)
        x_center = (x_bounds[1] - x_bounds[0]) / 2 + x_bounds[0]
        y_center = (y_bounds[1] - y_bounds[0]) / 2 + y_bounds[0]
        print("X_bounds:")
        print(x_bounds)
        print("Y_bounds:")
        print(y_bounds)
        for vertex in vertices:
            if x_center - radius < vertex[0] < x_center + radius and y_center - radius < vertex[1] < y_center + radius:
                    center_vertices.append(vertex)
        return center_vertices

    def get_bounds(self, vertices, dimension):
        first_vertex = vertices[0]
        bounds = [first_vertex[dimension], first_vertex[dimension]]
        for vertex in vertices:
            if vertex[dimension] < bounds[0]:
                bounds[0] = vertex[dimension]
            elif vertex[dimension] > bounds[1]:
                bounds[1] = vertex[dimension]
        return bounds

    def get_median_vertices(self, vertices):
        length = len(vertices)
        median_vertices = []
        for i in range(length):
            if length / 4 < i < length * .75:
                median_vertices.append(vertices[i])
        return median_vertices

    """
    TRIANGLES
    """

    def find_traversable_surface(self):
        triangles = self.mesh.triangles
        level_triangles = self.get_level_triangles(triangles)

    def get_level_triangles(self, triangles):
        level_triangles = []
        for triangle in triangles:
            if self.is_triangle_traversable_by_angle(triangle):
                level_triangles.append(triangle)
        return level_triangles

    def get_triangle_normal(self, triangle):
        # each point in the given triangle (the triangle holds indexes of vertexes, not vertexes themselves)
        p1 = self.mesh.vertices[int(triangle[0]) - 1]
        p2 = self.mesh.vertices[int(triangle[1]) - 1]
        p3 = self.mesh.vertices[int(triangle[2]) - 1]

        # vector 1 is from point 1 to 2
        u = self.make_vector(p1, p2)
        # vector 2 is from point 1 to 3
        v = self.make_vector(p1, p3)
        normal = [0, 0, 0]
        normal[0] = u[1] * v[2] - v[1] * u[2]
        normal[1] = -1 * (u[0] * v[2] - v[0] * u[2])
        normal[2] = u[0] * v[1] - v[0] * u[1]
        length = math.sqrt(math.pow(normal[0], 2) + math.pow(normal[1], 2) + math.pow(normal[2], 2))
        return [normal[0] / length, normal[1] / length, normal[2] / length]

    def is_triangle_traversable_by_z_length(self, triangle):
        normal = self.get_triangle_normal(triangle)
        return normal[2] > TRIANGLE_NORMAL_Z_TRAVERSABLE

    def angle_between_vectors(self, v1, v2):
        dot = float(v1[0]) * float(v2[0]) + float(v1[1]) * float(v2[1]) + float(v1[2]) * float(v2[2])
        lv1 = math.sqrt(float(math.pow(v1[0], 2) + math.pow(v1[1], 2) + math.pow(v1[2], 2)))
        lv2 = math.sqrt(float(math.pow(v2[0], 2) + math.pow(v2[1], 2) + math.pow(v2[2], 2)))
        x = dot / (lv1 * lv2)
        return math.acos(x)

    def is_triangle_traversable_by_angle(self, triangle):
        normal = self.get_triangle_normal(triangle)
        vertical = [0.0, 0.0, 1.0]
        angle = self.angle_between_vectors(vertical, normal)
        #print("normal: " + str(normal) + " vertical: " + str(vertical))
        #print("angle: " + str(angle))
        return math.fabs(angle) < TRIANGLE_NORMAL_ANGLE_TRAVERSABLE

    def get_centroid(self, triangle):
        v1 = self.mesh.vertices[int(triangle[0]) - 1]
        v2 = self.mesh.vertices[int(triangle[1]) - 1]
        v3 = self.mesh.vertices[int(triangle[2]) - 1]

        x = (v1[0] + v2[0] + v3[0]) / 3.0
        y = (v1[1] + v2[1] + v3[1]) / 3.0
        z = (v1[2] + v2[2] + v3[2]) / 3.0
        return [x, y, z]

    def is_point_in_triangle(self, point, triangle):
        p1 = self.mesh.vertices[int(triangle[0]) - 1]
        p2 = self.mesh.vertices[int(triangle[1]) - 1]
        p3 = self.mesh.vertices[int(triangle[2]) - 1]

        if point[0] > p1[0] and point[0] > p2[0] and point[0] > p3[0]:
            return False
        if point[0] < p1[0] and point[0] < p2[0] and point[0] < p3[0]:
            return False
        if point[1] > p1[1] and point[1] > p2[1] and point[1] > p3[1]:
            return False
        if point[1] < p1[1] and point[1] < p2[1] and point[1] < p3[1]:
            return False

        A = self.area_of_triangle(p1, p2, p3)
        t0 = self.area_of_triangle(point, p1, p2)
        t1 = self.area_of_triangle(point, p2, p3)
        t2 = self.area_of_triangle(point, p1, p3)

        difference_in_area = math.fabs(A - t0 - t1 - t2)

        return difference_in_area < 0.0005

    def is_point_in_triangle_simple(self, point, p1, p2, p3):
        """
        if (point[0] > p1[0] and point[0] > p2[0] and point[0] > p3[0]) or \
                (point[0] < p1[0] and point[0] < p2[0] and point[0] < p3[0]) or \
                (point[1] > p1[1] and point[1] > p2[1] and point[1] > p3[1]) or \
                (point[1] < p1[1] and point[1] < p2[1] and point[1] < p3[1]):
            return False
        """
        if point[0] > p1[0] and point[0] > p2[0] and point[0] > p3[0]:
            print("above x")
            return False
        if point[0] < p1[0] and point[0] < p2[0] and point[0] < p3[0]:
            print("below x")
            return False
        if point[1] > p1[1] and point[1] > p2[1] and point[1] > p3[1]:
            print("above y")
            return False
        if point[1] < p1[1] and point[1] < p2[1] and point[1] < p3[1]:
            print("below y")
            return False

        A = self.area_of_triangle(p1, p2, p3)
        t0 = self.area_of_triangle(point, p1, p2)
        t1 = self.area_of_triangle(point, p2, p3)
        t2 = self.area_of_triangle(point, p1, p3)

        difference_in_area = math.fabs(A - t0 - t1 - t2)

        return difference_in_area < 0.0005

    def area_of_triangle(self, p1, p2, p3):
        return math.fabs(
            (p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]))
            / 2
        )

    def get_triangle_from_point(self, p, triangles):
        #print("triangle for " + str(p) + "...")
        for triangle in triangles:
            if self.is_point_in_triangle(p, triangle):
                return triangle
        return [-1, -1, -1]

    """
    ZONES
    """

    def get_triangles_in_zone(self, zone):
        triangles_in_zone = []
        for triangle in self.mesh.triangles:
            point = self.get_centroid(triangle)
            if self.is_point_in_zone(zone, point):
                triangles_in_zone.append(triangle)

        return triangles_in_zone

    def is_point_in_zone(self, zone, point):
        top_left = zone.corners[0]
        top_right = zone.corners[1]
        bottom_right = zone.corners[2]
        bottom_left = zone.corners[3]

        area = math.fabs(top_left[0] - top_right[0]) * math.fabs(top_left[1] - bottom_left[1])
        # area of 4 triangles
        t0 = .5 * (point[0] * (top_left[1] - top_right[1]) +
                   top_left[0] * (top_right[1] - point[1]) +
                   top_right[0] * (point[1] - top_left[1]))

        t1 = .5 * (point[0] * (top_right[1] - bottom_right[1]) +
                   top_right[0] * (bottom_right[1] - point[1]) +
                   bottom_right[0] * (point[1] - top_right[1]))

        t2 = .5 * (point[0] * (bottom_right[1] - bottom_left[1]) +
                   bottom_right[0] * (bottom_left[1] - point[1]) +
                   bottom_left[0] * (point[1] - bottom_right[1]))

        t3 = .5 * (point[0] * (bottom_left[1] - top_left[1]) +
                   bottom_left[0] * (top_left[1] - point[1]) +
                   top_left[0] * (point[1] - bottom_left[1]))

        sum_of_triangles = math.fabs(t0) + math.fabs(t1) + math.fabs(t2) + math.fabs(t3)
        difference_in_area = math.fabs(sum_of_triangles - area)
        return difference_in_area < 0.05


    def get_traversable_mesh_in_zone(self, zone):
        potential_triangles = self.get_triangles_in_zone(zone)
        traversable_triangles = []
        for triangle in potential_triangles:
            if self.is_triangle_traversable_by_angle(triangle):
                traversable_triangles.append(triangle)

        return traversable_triangles

    # this is an old method DO NOT USE
    def make_occupancy_grid(self, area_corners):
        tl = area_corners[0]
        tr = area_corners[1]

        OG = []

        dx = tr[0] - tl[0]
        dy = tr[1] - tl[1]

        yi = 0
        traversable_triangles = []
        print("filtering triangles")
        for triangle in self.mesh.triangles:
            if self.is_triangle_traversable_by_angle(triangle):
                traversable_triangles.append(triangle)
        print("Number of triangles: " + str(len(self.mesh.triangles)) + " Number of TT: " + str(len(traversable_triangles)))
        print("number of filtered triangles " + str(len(self.mesh.triangles) - len(traversable_triangles)))
        while yi < OG_WIDTH:
            xi = 0
            while xi < OG_WIDTH:
                x = tl[0] + xi * dx / (OG_WIDTH - 1)
                y = tl[1] + yi * dy / (OG_WIDTH - 1)
                t = self.get_triangle_from_point([x, y], traversable_triangles)
                if t[0] == -1 and t[1] == -1 and t[2] == -1:
                    #print("triangle not found")
                    OG.append(-1)
                else:
                    print("triangle is traversable")
                    OG.append(self.get_triangle_normal(t)[2])
                xi += 1
            yi += 1
        return OG

    def make_occupancy_grid_in_front(self, visible_zones, area_corners):
        zone_triangles = dict()
        for zone in visible_zones:
            zone_triangles[zone.id] = []
        for triangle in self.mesh.triangles:
            if self.is_triangle_traversable_by_angle(triangle):
                centroid = self.get_centroid(triangle)
                for zone in visible_zones:
                    if self.is_point_in_zone(zone, centroid):
                        triangles = zone_triangles[zone.id]
                        triangles.append(triangle)
                        zone_triangles[zone.id] = triangles
                        break

        for zone in visible_zones:
            print("triangles in zone " + str(zone.id) + ": " + str(len(zone_triangles[zone.id])))

        tl = area_corners[0]
        tr = area_corners[1]
        br = area_corners[2]

        OG = []

        dx = tr[0] - tl[0]
        dy = tr[1] - br[1]
        total_not_found = 0
        total_no_zone = 0
        total_found = 0
        yi = 0
        while yi < OG_WIDTH:
            xi = 0
            while xi < OG_WIDTH:
                x = tl[0] + xi * dx / (OG_WIDTH - 1)
                y = tl[1] - yi * dy / (OG_WIDTH - 1)

                zone_id = -1
                for zone in visible_zones:
                    if self.is_point_in_zone(zone, [x, y]):
                        zone_id = zone.id
                        break
                if zone_id == -1:
                    total_no_zone += 1
                    OG.append(-1)
                else:
                    t = self.get_triangle_from_point([x, y], zone_triangles[zone_id])
                    if t[0] == -1 and t[1] == -1 and t[2] == -1:
                        total_not_found += 1
                        OG.append(-1)
                    else:
                        total_found += 1
                        OG.append(self.get_triangle_normal(t)[2])
                xi += 1
            yi += 1

        print("total triangles not found: " + str(total_not_found) + " total no zones: " + str(total_no_zone) + " total found: " + str(total_found))
        return OG