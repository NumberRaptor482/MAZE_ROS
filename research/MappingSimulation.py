from math import cos, inf, pi, sin
import time
import os

class MapSimulation:
    def __init__(self, width, height, start, end):
        self.width = width
        self.height = height
        self.start = start
        self.end = end
        self.walls = set()
        self.annotations = {}
        self.bot_position = self.start
        self.total_steps = 0
        self.last_steps = []

    def reset(self):
        self.bot_position = self.start
        self.total_steps = 0
        self.last_steps = []

    def _check_wall_intersects_with_line(self, line_start, line_end, wall_to_check) -> bool:
        x1, y1 = wall_to_check[0]
        x2, y2 = wall_to_check[1]

        sx, sy = line_start
        ex, ey = line_end

        if x1 == x2: # vertical wall
            # line segment is completely over or completely under the wall
            if (sy > max(y1, y2) and ey > max(y1, y2)) or (sy < min(y1, y2) and ey < min(y1, y2)):
                return False
            
            # line segment is completely to the left or right of the wall
            if (sx > x1 and ex > x1) or (sx < x1 and ex < x1):
                return False

            slope = (ey - sy) / (ex - sx)
            ix = x1
            iy = slope * (x1 - sx) + sy
            if min(y1, y2) < iy < max(y1, y2):
                return True
            
            return False
        else: # horizontal wall
            # line segment is completely over or completely under the wall
            if sy > y1 and ey > y1 or sy < y1 and ey < y1:
                return False
            
            # line segment is completely to the left or right of the wall
            if (sx > max(x1, x2) and ex > max(x1, x2)) or (sx < min(x1, x2) and ex < min(x1, x2)):
                return False
            
            slope = (ey - sy) / (ex - sx)
            iy = y1
            ix = (iy - sy) / slope + sx
            if min(x1, x2) < ix < max(x1, x2):
                return True
            return False

    def scan_lidar_basic(self) -> tuple[int, int, int, int]:
        """Returns the number of open blocks in each direction of the robot.
            The order is counter clockwise starting with the GLOBAL right direction."""

        raw_scan = self.scan_lidar_raw(points=4)

        return tuple(map(int, raw_scan))

    # 0 -> right (+x)
    # 1 -> up (-y)
    # 2 -> left (-x)
    # 3 -> down (+y)
    def move(self, direction) -> bool:
        diff = ((1, 0), (0, -1), (-1, 0), (0, 1))[direction]

        new_pos = self.bot_position[0] + diff[0], self.bot_position[1] + diff[1]

        if self.scan_lidar_basic()[direction] == 0:
            return False

        if len(self.last_steps) > 9:
            self.last_steps.pop(0)
        self.last_steps.append(self.bot_position)

        self.total_steps += 1

        self.bot_position = new_pos
        return True
        
    def scan_lidar_raw(self, points=360, max_distance=10)->tuple[float]:
        output = [inf] * points

        angle_inc = -2 * pi / points

        bx = self.bot_position[0] + 0.5
        by = self.bot_position[1] + 0.5

        angle = 2 * pi
        while angle >= 0:
            rex = cos(angle) * max_distance
            rey = sin(angle) * max_distance

            for wall in self.walls:
                (x1, y1), (x2, y2) = wall

                if x1 == x2: # vertical wall

                    if rex == 0:
                        continue
                    t = (x1 - bx) / rex
                    if t < 0 or t > 1:
                        continue
                    y_intersect = by + t * rey
                    if min(y1, y2) <= y_intersect <= max(y1, y2):
                        distance = t * max_distance
                        index = int(angle / angle_inc) % points
                        output[index] = min(output[index], distance)
                else: # horizontal wall
                    if rey == 0:
                        continue
                    t = (y1 - by) / rey
                    if t < 0 or t > 1:
                        continue
                    x_intersect = bx + t * rex
                    if min(x1, x2) <= x_intersect <= max(x1, x2):
                        distance = t * max_distance
                        index = int(angle / angle_inc) % points
                        output[index] = min(output[index], distance)

            angle += angle_inc
        return tuple(output)

    def add_wall(self, x1, y1, x2, y2):
        self.walls.add(((x1, y1), (x2, y2)))

    def is_done(self):
        return self.bot_position == self.end

    def get_position(self):
        return self.bot_position

    def print_map(self):
        buffer = []

        # print top border
        top_border = "+" + "   +" * self.width
        buffer.append(top_border)

        # print each row
        for y in range(self.height - 1):
            # print side walls and cells
            buffer.append(" " + " " * (self.width * 4 - 1) + " ")
            buffer.append('+' + '   +' * self.width)

        buffer.append(" " + " " * (self.width * 4 - 1) + " ")

        # print bottom border
        bottom_border = "+" + "   +" * self.width
        buffer.append(bottom_border)

        for i in range(len(buffer)):
            buffer[i] = list(buffer[i])

        for wall in self.walls:
            x1, y1 = wall[0]
            x2, y2 = wall[1]
            if y1 != y2:
                for y in range(min(y1, y2) + 1, max(y1, y2) + 1):
                    buffer[y * 2 - 1][x1 * 4] = '|'
            else:
                for x in range(min(x1, x2), max(x1, x2)):
                    buffer[y1 * 2][x * 4 + 1] = '-'
                    buffer[y1 * 2][x * 4 + 2] = '-'
                    buffer[y1 * 2][x * 4 + 3] = '-'


        buffer[self.start[1] * 2 + 1][self.start[0] * 4 + 2] = 'S'

        buffer[self.end[1] * 2 + 1][self.end[0] * 4 + 2] = 'E'


        # prints the last 10 spots it was at
        #for i, step in enumerate(self.last_steps):
        #    buffer[step[1] * 2 + 1][step[0] * 4 + 2] = str(i)

        buffer[self.bot_position[1] * 2 + 1][self.bot_position[0] * 4 + 2] = 'R'

        print("Total steps:", self.total_steps)
        for line in buffer:
            print(''.join(line))


def load_maze(file_name):
    with open(file_name, 'r') as f:
        lines = f.readlines()
        width, height = map(int, lines[0].strip().split())
        start = tuple(map(int, lines[1].strip().split()))
        end = tuple(map(int, lines[2].strip().split()))
        sim = MapSimulation(width, height, start, end)

        for line in lines[3:]:
            x1, x2, y1, y2 = map(int, line.strip().split())
            sim.add_wall(x1, y1, x2, y2)

    return sim