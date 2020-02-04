#!/usr/bin/env python3
import math
import pyzed.sl as sl
import time

from baseBot.MapMaker import MapMaker
from baseBot.MeshAnalyzer import MeshAnalyzer
from support.Constants import *

def mesh_test():
    mapMaker = MapMaker()
    timer = 0

    print("Getting Frames...")
    # Grab 500 frames and stop
    while timer < 500:
        mapMaker.update_map_async()
        if mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1
    meshAnalyzer = MeshAnalyzer(mapMaker.mesh)
    avg_height_before = meshAnalyzer.get_avg_height(meshAnalyzer.mesh.vertices)
    ordered = meshAnalyzer.sort_by_height(meshAnalyzer.mesh.vertices)
    median = meshAnalyzer.get_median_vertices(ordered)
    avg_height = meshAnalyzer.get_avg_height(median)
    print("Avg Height Before:")
    print(avg_height_before)
    print("Ordered:")
    print(len(ordered))
    print("Median:")
    print(len(median))
    print("Avg Height After:")
    print(avg_height)
    print("Lowest Vertex:")
    print(meshAnalyzer.get_lowest_vertex(meshAnalyzer.mesh.vertices))
    print("Number of Vertices within .2 height:")
    print(len(meshAnalyzer.get_lowest_vertices(meshAnalyzer.mesh.vertices, .2)))
    print("Total vertices:")
    print(len(meshAnalyzer.mesh.vertices))
    print("Center vertices:")
    center_vertices = meshAnalyzer.center_vertices(meshAnalyzer.mesh.vertices, .1)
    print(len(center_vertices))
    print("Avg Height of Center")
    print(meshAnalyzer.get_avg_height(center_vertices))


def plane_test():
    mapMaker = MapMaker()
    timer = 0

    print("Getting Frames...")
    # Grab 500 frames and stop
    while timer < 500:
        mapMaker.update_map_async()
        if mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1
    print("Getting ground mesh...")
    mesh = mapMaker.get_ground_mesh()
    mesh.filter(mapMaker.filter_params)
    mesh.apply_texture()
    print("Saving...")
    mesh.save("ground.obj")
    print("Saving Whole Mesh...")
    # Filter the mesh
    mapMaker.mesh.filter(mapMaker.filter_params)  # not available for fused point cloud
    # Apply the texture
    mapMaker.mesh.apply_texture()  # not available for fused point cloud
    # Save the mesh in .obj format
    mapMaker.mesh.save("mesh.obj")
    print("Done")


def map_maker_test():
    mapMaker = MapMaker()

    timer = 0

    print("Getting Frames...")
    # Grab 500 frames and stop
    start = time.time()
    while timer < 500:
        mapMaker.update_map_async()
        if mapMaker.get_frame() == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1
    elapsed = time.time() - start
    print("Elapsed Time:")
    print(elapsed)
    print("Saving...")
    # Filter the mesh
    mapMaker.mesh.filter(mapMaker.filter_params)  # not available for fused point cloud
    # Apply the texture
    mapMaker.mesh.apply_texture()  # not available for fused point cloud
    # Save the mesh in .obj format
    mapMaker.mesh.save("mesh.obj")
    print("Done")

def test():
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720  # Use HD720 video mode (default fps: 60)
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP  # Use a right-handed Y-up coordinate system
    init_params.coordinate_units = sl.UNIT.UNIT_METER  # Set units in meters

    zed.open(init_params)

    # Configure spatial mapping parameters
    mapping_parameters = sl.SpatialMappingParameters(sl.MAPPING_RESOLUTION.MAPPING_RESOLUTION_LOW,
                                                     sl.MAPPING_RANGE.MAPPING_RANGE_FAR)
    mapping_parameters.map_type = sl.SPATIAL_MAP_TYPE.SPATIAL_MAP_TYPE_MESH
    mapping_parameters.save_texture = True
    filter_params = sl.MeshFilterParameters()  # not available for fused point cloud
    filter_params.set(sl.MESH_FILTER.MESH_FILTER_LOW)  # not available for fused point cloud

    # Enable tracking and mapping
    py_transform = sl.Transform()
    tracking_parameters = sl.TrackingParameters(init_pos=py_transform)
    zed.enable_tracking(tracking_parameters)
    zed.enable_spatial_mapping(mapping_parameters)

    mesh = sl.Mesh()  # Create a mesh object
    timer = 0
    runtime_parameters = sl.RuntimeParameters()

    print("Getting Frames...")
    # Grab 500 frames and stop
    while timer < 500:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1

    print("Saving...")
    # Retrieve the spatial map
    zed.extract_whole_spatial_map(mesh)
    # Filter the mesh
    mesh.filter(filter_params)  # not available for fused point cloud
    # Apply the texture
    mesh.apply_texture()  # not available for fused point cloud
    # Save the mesh in .obj format
    mesh.save("mesh.obj")
    print("Done")

if __name__ == '__main__':
    #test()
    #map_maker_test()
    #plane_test()
    mesh_test()