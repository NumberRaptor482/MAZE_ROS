#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import MapMetaData, OccupancyGrid
from math import sin, cos, pi, atan2
from geometry_msgs.msg import PoseStamped, Pose
import geometry_msgs.msg
from interfaces.msg import Step

def check_grid(grid, x, y):
    """returns true if this pixel is a wall and false otherwise"""
    if x >= 0 and x < grid.info.width and y >= 0 and y < grid.info.height and grid.data[x + y * grid.info.width] > 50 and grid.data[x + y * grid.info.width] != 101: return True
    return False


def yaw_from_quat(q):
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
        self.width = width
        self.height = height
        self.grid = data
        self.path = []  # replan on next operation

    def _is_adjacent(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


    def send_next_operation(self):
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
        """Pop and combine consecutive path points moving in the same direction."""
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
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path

    # -------- Grid utils (ROW-MAJOR) --------

    def _idx(self, x, y):
        # row-major: index = y*width + x
        return y * self.width + x

    def _in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def _is_border(self, x, y):
        return x == 0 or y == 0 or x == self.width - 1 or y == self.height - 1

    def _is_blocked(self, x, y):
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
        # row-major scan
        for y in range(self.height):
            base = y * self.width
            for x in range(self.width):
                if self.grid[base + x] == ROBOT:
                    return (x, y)
        return None

    # -------- Operation mapping --------

    def _dir_from_to(self, a, b):
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
        if direction is None:
            return

        magnitude = 1
        if isinstance(direction, tuple):
            direction, magnitude = direction

        step = Step()
        
        print('directoion', direction)

        if direction == "LEFT": step.xstep = int(magnitude)
        elif direction == "RIGHT": step.xstep = -int(magnitude)
        elif direction == "UP": step.ystep = -int(magnitude)
        elif direction == "DOWN": step.ystep = int(magnitude)
        else: return

        print(f'publishing {step}')

        self.pub.publish(step)

    def plan_next_step(self, current_pose):
        self.execute_step_from_program(self.explorer.send_next_operation())

    def send_next_operation(self):
        self.execute_step_from_program(self.explorer.send_next_operation())

    def updated_map_received(self, width, height, data):
        self.explorer.updated_map_received(width, height, data)

    def get_walls(self, walls):
        if True:
            print('\n')
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