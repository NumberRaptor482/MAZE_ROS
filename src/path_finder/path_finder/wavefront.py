#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import MapMetaData, OccupancyGrid
from math import sin, cos, pi, atan2
from geometry_msgs.msg import PoseStamped, Pose
import geometry_msgs.msg
from interfaces.msg import Step

def check_grid(grid, x, y):
    """returns true if this pixel is a wall and false otherwise
    
    Args:
        grid: An ROS occupancy grid.
        x: The x-coordinate to be checked.
        y: The y-coordinate to be checked.

    Returns: True if the position is a valid point on the maze and the cells occupancy value is greater than 50 and the value is not 101.
        
    """
    if x >= 0 and x < grid.info.width and y >= 0 and y < grid.info.height and grid.data[x + y * grid.info.width] > 50 and grid.data[x + y * grid.info.width] != 101: return True
    return False


def yaw_from_quat(q):
    """Calculates the yaw (rotation around the z-axis) from a quaternion.

    Args:
        q: The quaternion representing the orientation.

    Returns:
        The orientation in radians.
    """
    # geometry_msgs/Quaternion -> yaw (rad)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)

from collections import deque
from heapq import heappush, heappop
from math import inf

WALL = 100
EMPTY = 0
UNKNOWN = -1
ROBOT = 101
MAP_UNITS_PER_STEP = 2

class FrontierExplorer:
    """A ROS 2 node for the frontier explorer. 

    Attributes:
        allow_unknown: a bool for whether to allow unknown.
        y_increases_down: a bool for whether y increases down.
        width: width of the known map.
        height: height of the known map.
        grid: the map grid.
        path: the path of the robot to take.
    """
    def __init__(self, allow_unknown=False, y_increases_down=True):
        """
        allow_unknown: if True, A* may path through UNKNOWN cells (usually False for exploration).
        y_increases_down:
            True  => "DOWN" means (x, y+1)  (typical grid/ASCII display)
            False => "DOWN" means (x, y-1)  (Cartesian-style)
        """
        self.allow_unknown = allow_unknown
        self.y_increases_down = y_increases_down

        self.width = None
        self.height = None
        self.grid = None  # 1D, row-major (y*width + x)
        self.path = []

    def updated_map_received(self, width, height, data):
        """Updates the map width, height, and grid data. Also clears the path list.

        Args:
            width: width of the known map.
            height: height of the known map.
            grid: the map grid.
        """
        self.width = width
        self.height = height
        self.grid = data
        self.path = []  # replan on next operation

    def _is_adjacent(self, a, b):
        """Calculates whether or not a and b coordinates are adjacent.

        Args:
            a: the first coordinate tuple.
            b: the second coordinate tuple.

        Returns:
        A boolean of whether or not a and b are 1 away from each other either vertically or horizontally, but not diagonally.
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


    def send_next_operation(self):
        """Main decision-making function for the robot's next movement. Tries to follow its existing path first, and if that path is no longer valid, it replans.

        Returns:
        None if the grid is empty, cannot find robot's current position, no reachable frontier exists, or if A* failed or its returned path doesn't contain actual movement.
        Returns the next movement operation processed by extracting the first straight section while removing robot's current position from self.path.
        """
        if self.grid is None:
            return None

        start = self._find_robot()
        if start is None:
            return None

        # Follow existing path only if next step is still adjacent and free
        if self.path:
            operation = self._consume_straight_segment(start)
            if operation is not None:
                return operation
            self.path = []  # stale/desynced path => replan

        # Choose closest frontier by wavefront/BFS
        frontier = self._choose_frontier_wavefront(start)
        if frontier is None:
            return None

        # A* path to chosen frontier
        full_path = self._astar(start, frontier)
        if not full_path or len(full_path) < 2:
            return None

        self.path = full_path[1:]
        return self._consume_straight_segment(start)

    def _consume_straight_segment(self, start):
        """Pop and combine consecutive path points moving in the same direction.
        
        Args:
            start: The starting node.

        Returns:
        None if there is no path, whether the first coordinate in the planned path is not adjacent or is blocked, or if no valid cells were found.
        Otherwise, returns a command as a tuple(direction, step_count) while removing the processed cells from self.path.
        """
        if not self.path:
            return None

        prev = start
        first = self.path[0]
        if not self._is_adjacent(prev, first) or self._is_blocked(*first):
            return None

        direction = self._dir_from_to(prev, first)
        run_cells = 0
        run_end = start

        for nxt in self.path:
            if not self._is_adjacent(prev, nxt) or self._is_blocked(*nxt):
                break

            step_dir = self._dir_from_to(prev, nxt)
            if step_dir != direction:
                break

            run_cells += 1
            run_end = nxt
            prev = nxt

        if run_cells == 0:
            return None

        # Path coordinates are 1-unit map cells, while motion steps are 2-unit blocks.
        # Convert map-cell run length to mover step count so wall-resolution cells
        # don't inflate command magnitude.
        step_count = max(1, run_cells // MAP_UNITS_PER_STEP)

        consumed_cells = step_count * MAP_UNITS_PER_STEP
        if run_cells < MAP_UNITS_PER_STEP:
            consumed_cells = 1

        self.path = self.path[consumed_cells:]
        return (direction, step_count)

    # -------- Frontier selection (wavefront) --------

    def _choose_frontier_wavefront(self, start):
        """BFS to explore the graph layer by layer, AKA wavefront propagation. 

        Args:
            start: The starting node.

        Returns:
        The closest frontier it found which is coordinate as tuple.
        """
        q = deque([start])
        dist = {start: 0}

        best = None
        best_d = inf

        while q:
            x, y = q.popleft()
            d = dist[(x, y)]

            if d < best_d and self._is_frontier(x, y):
                best = (x, y)
                best_d = d

            if d > best_d:
                continue

            for nx, ny in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                if (nx, ny) in dist:
                    continue
                if self._is_blocked(nx, ny):
                    continue
                dist[(nx, ny)] = d + 1
                q.append((nx, ny))

        return best

    def _is_frontier(self, x, y):
        """Checks if a coordinate's node is a frontier node.

        Args:
            x: The x-coordinate.
            y: The y-coordinate.

        Returns:
        False if the cell is blocked or the cell is neither EMPTY or ROBOT. Returns true if any valid cell has a neighboring(not diagonal) UNKNOWN cell.
        """
        if self._is_blocked(x, y):
            return False
        v = self.grid[self._idx(x, y)]
        if v not in (EMPTY, ROBOT):
            return False

        # frontier: adjacent to at least one UNKNOWN cell (not counting border as valid)
        for nx, ny in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
            if not self._in_bounds(nx, ny):
                continue
            if self._is_border(nx, ny):
                continue
            if self.grid[self._idx(nx, ny)] == UNKNOWN:
                return True
        return False

    # -------- A* --------

    def _astar(self, start, goal):
        """Applies the A* algorithm to get from a starting location to goal.

        Args:
            start: The starting coordinate(x,y).
            goal: The goal coordinate(x,y).

        Returns:
        None if the destination is blocked or there is no path. Otherwise, returns the path as a list of coordinates to get to goal from start.
        """
        if self._is_blocked(*goal):
            return None

        def h(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_heap = []
        heappush(open_heap, (h(start, goal), 0, start))  # (f, g, node)

        came_from = {}
        gscore = {start: 0}

        while open_heap:
            _, g, cur = heappop(open_heap)

            if cur == goal:
                return self._reconstruct(came_from, cur)

            if g != gscore.get(cur, inf):
                continue

            x, y = cur
            for nx, ny in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                if self._is_blocked(nx, ny):
                    continue
                ng = g + 1
                n = (nx, ny)
                if ng < gscore.get(n, inf):
                    came_from[n] = cur
                    gscore[n] = ng
                    heappush(open_heap, (ng + h(n, goal), ng, n))

        return None

    def _reconstruct(self, came_from, cur):
        """Reconstructs the path from a starting point to an ending point using a dictionary that holds where each position came from.

        Args:
            came_from: A dictionary that stores the predecessor of each node.
            cur: The current node(destination).

        Returns:
        A path as a list of coordinates that start from a starting node to the destination node.
        """
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path

    # -------- Grid utils (ROW-MAJOR) --------

    def _idx(self, x, y):
        """Converts 2D coordinates to 1D array index.

        Args:
            x: the x coordinate.
            y: the y coordinate.

        Returns:
        Returns the 1D array index.
        """
        # row-major: index = y*width + x
        return y * self.width + x

    def _in_bounds(self, x, y):
        """Checks to see if a coordinate is in bound of the map.

        Args:
            x: the x coordinate.
            y: the y coordinate.

        Returns:
        A boolean of whether or not both the x and the y coordinates are in bounds of the map.
        """
        return 0 <= x < self.width and 0 <= y < self.height

    def _is_border(self, x, y):
        """Checks to see if a coordinate is on the border of the map.

        Args:
            x: the x coordinate.
            y: the y coordinate.

        Returns:
        A boolean of whether or not x and y are on the border of the map.
        """
        return x == 0 or y == 0 or x == self.width - 1 or y == self.height - 1

    def _is_blocked(self, x, y):
        """Checks to see whether or not the robot is not capable of going to those coordinate, or in short, if the location is blocked.

        Args:
            x: the x coordinate.
            y: the y coordinate.

        Returns:
        True if the coordinate is out of bounds, a border of the maze, a wall, or not yet known. Returns false otherwise.
        """
        if not self._in_bounds(x, y):
            return True
        if self._is_border(x, y):  # wrapped in walls rule
            return True

        v = self.grid[self._idx(x, y)]
        if v == WALL:
            return True
        if v == UNKNOWN and not self.allow_unknown:
            return True
        return False

    def _find_robot(self):
        """Tries to find where the robot is in the map.

        Returns:
        The coordinates of the robot in the map in a tuple (x, y) or None if the robot cannot be found.
        """
        # row-major scan
        for y in range(self.height):
            base = y * self.width
            for x in range(self.width):
                if self.grid[base + x] == ROBOT:
                    return (x, y)
        return None

    # -------- Operation mapping --------

    def _dir_from_to(self, a, b):
        """Checks to see whether or not the robot is not capable of going to those coordinate, or in short, if the location is blocked.

        Args:
            a: The first coordinate tuple(from).
            b: The second coordinate tuple(to).

        Returns:
        The direction as a string("RIGHT", "LEFT", "DOWN", "UP") to get to the second coordinate from the first. 
        
        Raises:
        ValueError: If the coordinates a and b are not adjacent to each other.
        """
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay

        # convert dy to command depending on coordinate convention
        if dx == 1 and dy == 0:
            return "RIGHT"
        if dx == -1 and dy == 0:
            return "LEFT"

        if self.y_increases_down:
            if dx == 0 and dy == 1:
                return "DOWN"
            if dx == 0 and dy == -1:
                return "UP"
        else:
            if dx == 0 and dy == 1:
                return "UP"
            if dx == 0 and dy == -1:
                return "DOWN"

        raise ValueError(f"Non-adjacent step {a} -> {b}")

class Wavefront(Node):
    """A node that enables the wavefront aglorithm.

    Attributes:
        sub1: subscriber for the processed occupancy grid from /processed topic.
        sub2: subscriber for the pose from /ready topic.
        pub: publisher for step command to /step topic.
        explorer: FrontierExplorer object with allow_unknown=False.
    """
    def __init__(self):
        super().__init__('wavefront')
        self.sub1 = self.create_subscription(
            OccupancyGrid,
            '/processed',
            self.get_walls,
            10
        )

        self.sub2 = self.create_subscription(
            Pose,
            '/ready',
            self.plan_next_step,
            1
        )

        self.pub = self.create_publisher(
            Step,
            '/step',
            10
        )

        self.explorer = FrontierExplorer(allow_unknown=False)
        self.execute_step_from_program(self.explorer.send_next_operation())

    def execute_step_from_program(self, direction):
        """Converts the direction and an optional movement magnitude, if direction is passed as a tuple(direction, magnitude), to a ROS 2 Step message.
        The message is then published using self.pub.

        Args:
            direction: the direction as a string(LEFT, RIGHT, UP, DOWN).
        """
        if direction is None:
            return

        magnitude = 1
        if isinstance(direction, tuple):
            direction, magnitude = direction

        step = Step()
        
        print('direction', direction)

        if direction == "LEFT": step.xstep = int(magnitude)
        elif direction == "RIGHT": step.xstep = -int(magnitude)
        elif direction == "UP": step.ystep = -int(magnitude)
        elif direction == "DOWN": step.ystep = int(magnitude)
        else: return

        print(f'publishing {step}')

        self.pub.publish(step)

    def plan_next_step(self, current_pose):
        """Gets the next operations from FrontierExplorer, self.explorer, and calls the execute_step_from_program function.

        Args:
            current_pose: the current position of the robot(WARNING: is not used).
        """
        self.execute_step_from_program(self.explorer.send_next_operation())

    def send_next_operation(self):
        """Gets the next operations from FrontierExplorer, self.explorer, and calls the execute_step_from_program function."""
        self.execute_step_from_program(self.explorer.send_next_operation())

    def updated_map_received(self, width, height, data):
        """Takes the updated map and passes width, heigh, and data to the FrontierExplorer's, self.exlorer's,updated_map_received function.

        Args:
            width: The width of the updated map.
            height: The height of the updated map.
            data: The updated map.
        """
        self.explorer.updated_map_received(width, height, data)

    def get_walls(self, walls):
        """Prints a visualization of the processed map to the terminal. Passes the map info to the method updated_map_recieved.

        Args:
            walls: The OccupancyGrid message of the processed map.
        """
        #Why is there an 'if True' here?
        if True:
            print('\n')
            # is this right???? Shouldn't it be height for r and width for c?
            for r in range(walls.info.width):
                for c in range(walls.info.height):
                    e = walls.data[r*walls.info.width+c]
                    if e == 100:
                        e = 'X'
                    elif e == 0: e = ' '
                    elif e == -1: e = '?'
                    elif e == 101: e = 'R'
                    print(e, end='')
                print()
        self.updated_map_received(walls.info.width, walls.info.height, walls.data)

def main():
    rclpy.init()
    node = Wavefront()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()