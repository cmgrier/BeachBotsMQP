class Zone:
    def __init__(self, zone_corners, id):
        self.corners = zone_corners  # list of [top left, top right, bottom right, bottom left]
        self.id = id

    #   x ->
    # y 1  2  3  4
    # |    map
    # v 9 10 11 12
    def is_out_of_zone(self, position):
        if self.corners[1][0] > self.corners[2][0]:
            x_max = self.corners[1][0]
        else:
            x_max = self.corners[2][0]

        if self.corners[0][0] > self.corners[3][0]:
            x_min = self.corners[3][0]
        else:
            x_min = self.corners[0][0]

        if self.corners[2][1] > self.corners[3][1]:
            y_max = self.corners[2][1]
        else:
            y_max = self.corners[3][1]

        if self.corners[0][1] > self.corners[1][1]:
            y_min = self.corners[1][1]
        else:
            y_min = self.corners[0][1]

        return x_max > position[0] > x_min and y_max > position[1] > y_min
