stored_map_size = 10
safe_distance_between_bots = 100
avoid_distance = 30

zone_width = 20
landing_strip_width = 20

# ask Chris. Essentially we prioritize to avoiding robots by backing up first rather than trying to move forward
avoid_direction_priority_list = [3, 4, 2, 5, 1, 6, 0, 7]

#AStar
standard_move_cost = 1
difficult_terrain_factor = 1
turning_factor = 1

terrain_too_difficult = 30
zone_too_difficult_percent = .1
