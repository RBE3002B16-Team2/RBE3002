import copy
import rospy
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path


class CostmapThing:
    def __init__(self):
        self.onedmap = None

    def map_cb(self, og):
        self.og = og
        self.onedmap = list(og.data)

    def map_update_cb(self, ogu, publisher=None):
        if self.onedmap is None:
            raise Exception("global map update aborted because no initial map received")
        map_index = 0
        update_index = 0
        from_x = ogu.x
        from_y = ogu.y
        to_x = from_x + ogu.width - 1
        to_y = from_y + ogu.height - 1
        for row in range(0, self.og.info.height):
            for col in range(0, self.og.info.width):
                if from_x <= col <= to_x and from_y <= row <= to_y:
                    self.onedmap[map_index] = ogu.data[update_index]
                    update_index += 1
                map_index += 1
        if publisher is not None:
            publisher.publish(self.make_grid_cell())

    def make_grid_cell(self):
        resolution = self.og.info.resolution
        width = self.og.info.width
        height = self.og.info.height
        offsetX = self.og.info.origin.position.x
        offsetY = self.og.info.origin.position.y

        # resolution and offset of the map
        k = 0
        cells = GridCells()
        cells.header = self.og.header
        cells.cell_width = resolution
        cells.cell_height = resolution

        for i in range(0, height):  # height should be set to hieght of grid
            for j in range(0, width):  # width should be set to width of grid
                if self.onedmap[k] < 30:
                    cells.cells.append(getPoint((j, i), resolution, offsetX, offsetY))
                k += 1

        return cells


def getPoint(gridpos, resolution, offsetX, offsetY):
    point = Point()
    point.x = (gridpos[0] * resolution) + offsetX + (.5 * resolution)
    point.y = (gridpos[1] * resolution) + offsetY + (.5 * resolution)
    point.z = 0

    return point
