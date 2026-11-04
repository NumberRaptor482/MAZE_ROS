#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import MapMetaData, OccupancyGrid
from math import sin, cos, pi, atan2
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg

def check_grid(grid, x, y):
    """returns true if this pixel is a wall and false otherwise"""
    if x >= 0 and x < grid.info.width and y >= 0 and y < grid.info.height and grid.data[x + y * grid.info.width] > 50 and grid.data[x + y * grid.info.width] != 101: return True
    return False


def yaw_from_quat(q):
    # geometry_msgs/Quaternion -> yaw (rad)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class WallFinder(Node):
    def __init__(self):
        super().__init__('wallfinder')
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            '/processed',
            10
        )

        self.robot_pub = self.create_subscription(PoseStamped, 'tracked_pose', self.position_updater, 10)
        self.robot_pose = None
        self.visited_nodes = set()

    @staticmethod
    def to_node_coords(x, y):
        return int(x + 1.5), int(-y + 1.5)

    def position_updater(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y

        theta = yaw_from_quat(pose.pose.orientation)
        self.robot_pose = Pose(x, y, theta)
        node_x, node_y = self.to_node_coords(self.robot_pose.x, self.robot_pose.y)
        if 0 <= node_x < 11 and 0 <= node_y < 11:
            self.visited_nodes.add((node_x, node_y))


    def whats_in_this_spot(self, grid, x, y):#, output_image):
        all_unknown = True
        result = 0
        for dx in range(-9, 9):
            for dy in range(-9, 9):
                if 0 <= x + dx < grid.info.width and 0 <= y + dy < grid.info.height:
                    value = grid.data[x + dx + grid.info.width * (y + dy)]
                    if value != -1:
                        all_unknown = False
                    if value > 50:
                        result = 100
                    #output_image[x + dx + grid.info.width * (y + dy)] = 100
        
        if result == 100: return 100
        if all_unknown: return -1
        return 0


    def find_walls_again(self, grid):
        """Simplified approach that assumes perfect measurements"""
        w = grid.info.width
        h = grid.info.height
        data = grid.data
        box_size = int(1 / grid.info.resolution * 2.1) # 2.1 meters per box, resolution pixels per meter
        output = [] # 11x11. 0 = open, -1 = unknown, 1 = wall

        ox = -int(grid.info.origin.position.x / grid.info.resolution)
        oy = -int(grid.info.origin.position.y / grid.info.resolution)

        #output_image = []

        #for i in range(w * h): output_image.append(0)

        # take half box sized steps over all grid squares and check whats there
        for y in range(11):
            #line = []
            for x in range(11):
                output.append(self.whats_in_this_spot(grid, int((x - 1) * box_size // 2) + ox, int((-y + 1) * box_size // 2) + oy))#, output_image))
            #output.append(line)

        # Never treat any previously occupied robot node as a wall.
        for vx, vy in self.visited_nodes:
            if 0 <= vx < 11 and 0 <= vy < 11:
                output[vx + vy * 11] = 0

        if self.robot_pose is not None:
            x_unit, y_unit = self.to_node_coords(self.robot_pose.x, self.robot_pose.y)
            if 0 <= x_unit < 11 and 0 <= y_unit < 11:
                print(x_unit, y_unit)
                output[x_unit + y_unit*11] = 101

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info = MapMetaData()
        msg.info.resolution = 1.0
        msg.info.width = 11
        msg.info.height = 11
        msg.info.origin = geometry_msgs.msg.Pose()
        msg.data = output

        #for r in output:
        #    for c in r:
        #        print('X' if c == 1 else ('?' if c == -1 else ('R' if c == 2 else ' ')), end='')
        #    print()
        #print('\n\n')
        return msg



    def find_corners(self, grid):
        w = grid.info.width
        h = grid.info.height
        data = grid.data

        output = []


        # step 1 - convert to walls in image coordiantes
        # for now assume that walls are straight up or straight across
        for x in range(w):
            for y in range(h):
                base = y*w

                if not check_grid(grid, x, y):
                    continue

                # is it a horizontal wall
                if check_grid(grid, x + 3, y): #check_grid(grid, x + 5, y - 1) or check_grid(grid, x + 5, y) or check_grid(grid, x + 5, y + 1):
                    dx = x
                    while check_grid(grid, dx, y - 1) or check_grid(grid, dx, y) or check_grid(grid, dx, y + 1):
                        data[dx + base] = 101
                        #data[dx + base + y] = -1
                        #data[dx + base - y] = -1
                        dx += 1

                if check_grid(grid, x - 3, y):#  or check_grid(grid, x - 5, y) or check_grid(grid, x - 5, y + 1):
                    dx = x
                    while check_grid(grid, dx, y - 1) or check_grid(grid, dx, y) or check_grid(grid, dx, y + 1):
                        data[dx + base] = 101
                        #data[dx + base + y] = 101
                        #data[dx + base - y] = 101
                        dx -= 1
    
                    # is it a vertical wall
                if check_grid(grid, x, y + 3): #check_grid(grid, x + 5, y - 1) or check_grid(grid, x + 5, y) or check_grid(grid, x + 5, y + 1):
                    dy = y
                    while check_grid(grid, x - 1, dy) or check_grid(grid, x, dy) or check_grid(grid, x + 1, dy):
                        data[x + dy*w] = 101
                        #data[dx + base + y] = -1
                        #data[dx + base - y] = -1
                        dy += 1

                if check_grid(grid, x, y - 3): #check_grid(grid, x + 5, y - 1) or check_grid(grid, x + 5, y) or check_grid(grid, x + 5, y + 1):
                    dy = y
                    while check_grid(grid, x - 1, dy) or check_grid(grid, x, dy) or check_grid(grid, x + 1, dy):
                        data[x + dy*w] = 101
                        #data[dx + base + y] = -1
                        #data[dx + base - y] = -1
                        dy -= 1
                # otherwise not enough info so ignore for now. 

        for e in data:
            if e == 101:
                output.append(100)
            else: output.append(0)
            

        # step 2 - image coords to map coords


        # step 3 - map coord walls to wall array

        
        return output

    def cb(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        origin = msg.info.origin  # geometry_msgs/Pose

        # data is a flat array (row-major) of int8: -1 unknown, 0 free, 100 occupied
        data = msg.data

        self.get_logger().info(
            f"Map: {w}x{h}, res={res} m/cell, origin=({origin.position.x:.3f}, {origin.position.y:.3f})"
        )

        #out = OccupancyGrid()
        #out.info = msg.info
        #out.header = msg.header
        #out.data = self.find_walls_again(msg)

        walls = self.find_walls_again(msg)

        self.pub.publish(walls)#out)

def main():
    rclpy.init()
    node = WallFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()