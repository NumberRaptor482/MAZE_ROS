import json
from time import time
import pygame
import random
import sys
import heapq
import math
import numpy as np

# --- CONFIGURATION ---
WIDTH, HEIGHT = 150, 150
CELL_SIZE = 30
COLS = WIDTH // CELL_SIZE
ROWS = HEIGHT // CELL_SIZE
FPS = 60
total_steps = 0

# ROBOT PHYSICS CONSTANTS
MAX_LINEAR_SPEED = 1000.0  # Pixels per frame
MAX_ANGULAR_SPEED = 15 # Radians per frame
LIDAR_RANGE = 200 # Pixels (Continuous range, not cells)
LIDAR_RAYS = 36 # Lower ray count for performance in Python

# COLORS
COLOR_BG = (10, 10, 15)
COLOR_WALL = (50, 50, 60)
COLOR_FREE = (200, 200, 200)
COLOR_UNKNOWN = (20, 20, 30)
COLOR_ROBOT = (255, 215, 0)
COLOR_LIDAR = (255, 50, 50)
COLOR_PATH = (0, 191, 255)
COLOR_GOAL = (255, 0, 0)

# --- MOCK ROS MESSAGES ---
class Twist:
    """Standard ROS2 Message for Velocity"""
    def __init__(self):
        self.linear_x = 0.0
        self.angular_z = 0.0

class LaserScan:
    """Standard ROS2 Message for Lidar"""
    def __init__(self):
        self.ranges = [] # List of floats
        self.angle_min = 0.0
        self.angle_increment = 0.0

# --- THE SIMULATOR (GAZEBO REPLACEMENT) ---
class PhysicsNode:
    """
    Represents the 'Real World'. 
    Handles collisions, moves the robot based on physics, 
    and generates sensor data.
    """
    def __init__(self, maze_grid):
        self.grid = maze_grid # The Ground Truth
        # Robot State (Continuous, not grid-based)
        self.x = CELL_SIZE * 1.5
        self.y = CELL_SIZE * 1.5
        self.theta = 0.0 # Facing East
        
        # Velocity State
        self.v = 0.0
        self.w = 0.0
        
        # Mock Publisher/Subscriber Storage
        self.latest_cmd = Twist()

    def cmd_vel_callback(self, msg):
        """Receives motor commands from the Brain"""
        self.latest_cmd = msg

    def update_physics(self):
        """Applies kinematics: x_new = x + v * cos(theta)"""
        # Apply limits
        target_v = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, self.latest_cmd.linear_x))
        target_w = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, self.latest_cmd.angular_z))
        
        # Simple movement (In real ROS, we'd add inertia/acceleration)
        self.theta += target_w
        self.theta %= (2 * math.pi)
        
        new_x = self.x + target_v * math.cos(self.theta)
        new_y = self.y + target_v * math.sin(self.theta)
        
        # Collision Detection (Circle vs Grid Walls)
        robot_radius = CELL_SIZE / 3
        grid_x = int(new_x // CELL_SIZE)
        grid_y = int(new_y // CELL_SIZE)
        
        if 0 <= grid_x < COLS and 0 <= grid_y < ROWS:
            if self.grid[grid_y][grid_x] == 0: # 0 is Path
                self.x = new_x
                self.y = new_y
            else:
                # Bonk! Hit a wall. Stop.
                self.x = self.x # Don't move
                self.y = self.y
                # Optional: Slide along wall logic could go here

    def generate_scan(self):
        """Simulates the Lidar hardware. Returns a LaserScan message."""
        scan = LaserScan()
        scan.angle_min = 0
        scan.angle_increment = (2 * math.pi) / LIDAR_RAYS
        
        for i in range(LIDAR_RAYS):
            angle = self.theta + (i * scan.angle_increment)
            dist = 0
            found_wall = False
            
            # Raycast step size
            step = 4 
            eye_x, eye_y = self.x, self.y
            
            while dist < LIDAR_RANGE:
                eye_x += math.cos(angle) * step
                eye_y += math.sin(angle) * step
                dist += step
                
                gx, gy = int(eye_x // CELL_SIZE), int(eye_y // CELL_SIZE)
                
                if not (0 <= gx < COLS and 0 <= gy < ROWS):
                    break # Out of bounds
                
                if self.grid[gy][gx] == 1: # Hit Wall
                    found_wall = True
                    break
            
            # ROS convention: infinite/max range if no return
            if found_wall:
                scan.ranges.append(dist)
            else:
                scan.ranges.append(float('inf'))
                
        return scan

# --- THE ROBOT BRAIN (ROS NODE REPLACEMENT) ---
class NavigationNode:
    """
    The Intelligence. 
    1. Mapping: Subscribes to Scan -> Updates Grid
    2. Planning: Finds Frontiers -> Runs A*
    3. Control: A* Path -> Twist Message (PID)
    """
    def __init__(self, start_pos):
        # Internal Map (-1: Unknown, 0: Free, 1: Wall)
        self.map = [[-1 for _ in range(COLS)] for _ in range(ROWS)]
        self.current_pose = (start_pos[0], start_pos[1], 0) # x, y, theta
        
        self.path = []
        self.target_frontier = None
        self.finished = False

    def scan_callback(self, scan_msg, pose):
        """Inverse Sensor Model: Converts Ranges -> Map Probability"""
        self.current_pose = pose # Assume perfect localization for simplicity
        rx, ry, rtheta = pose
        
        # Mark robot position as free
        cx, cy = int(rx // CELL_SIZE), int(ry // CELL_SIZE)
        self.map[cy][cx] = 0
        
        for i, dist in enumerate(scan_msg.ranges):
            angle = rtheta + (i * scan_msg.angle_increment)
            
            # If infinite, clamp to max range for clearing space
            if dist == float('inf'):
                calc_dist = LIDAR_RANGE
                hit = False
            else:
                calc_dist = dist
                hit = True
            
            # Walk the ray to clear free space
            steps = int(calc_dist / CELL_SIZE)
            for s in range(steps):
                # Calculate intermediate point
                dx = math.cos(angle) * (s * CELL_SIZE)
                dy = math.sin(angle) * (s * CELL_SIZE)
                gx = int((rx + dx) // CELL_SIZE)
                gy = int((ry + dy) // CELL_SIZE)
                
                if 0 <= gx < COLS and 0 <= gy < ROWS:
                    if self.map[gy][gx] == -1: # Only update unknowns
                        self.map[gy][gx] = 0
            
            # Mark the wall hit
            if hit:
                wx = rx + math.cos(angle) * dist
                wy = ry + math.sin(angle) * dist
                wgx, wgy = int(wx // CELL_SIZE), int(wy // CELL_SIZE)
                if 0 <= wgx < COLS and 0 <= wgy < ROWS:
                    self.map[wgy][wgx] = 1

    def compute_cmd_vel(self):
        """
        The Controller.
        Converts the A* path (list of points) into a Twist message.
        Uses a simple 'Pure Pursuit' or 'Turn-then-Drive' logic.
        """
        cmd = Twist()
        
        if not self.path or self.finished:
            return cmd # Stop (0,0)
        
        # Get target cell center in pixels
        target_cell = self.path[0]

        if self.map[target_cell[0]][target_cell[1]] == 1:
            # Target is a wall (shouldn't happen), pop and wait
            self.path = [] # reset path to trigger replan

            print("Found wall on path. Rerouting...")
            return cmd

        tx = target_cell[1] * CELL_SIZE + CELL_SIZE/2
        ty = target_cell[0] * CELL_SIZE + CELL_SIZE/2
        
        rx, ry, rtheta = self.current_pose
        
        # Calculate Error
        dx = tx - rx
        dy = ty - ry
        dist = math.sqrt(dx**2 + dy**2)
        
        # Angle to target
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - rtheta
        
        # Normalize angle (-pi to pi)
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi
        
        # --- PID LOGIC ---
        
        # 1. If we are close to the waypoint, pop it
        if dist < 10: 
            self.path.pop(0)
            global total_steps
            total_steps += 1
            return cmd # Brief pause
            
        # 2. If facing wrong way, Turn in place
        if abs(angle_diff) > 0.5: # > 30 degrees
            cmd.angular_z = 0.5 * np.sign(angle_diff) # Turn speed
            cmd.linear_x = 0.0
        else:
            # 3. Drive and steer
            cmd.linear_x = 5 # Drive speed
            cmd.angular_z = 0.8 * angle_diff # Proportional steering
            
        return cmd
    
    def maze_to_file(self, file_path):
        json.dump(self.map, open(file_path, 'w'))

    def plan_path(self):
        """Finds nearest frontier and runs A*"""
        def plan_path(self):
            """Finds nearest frontier and runs A*"""
            if self.path: return # Already have a plan
            
            # Identify Frontiers
            frontiers = []
            for r in range(1, ROWS - 1):
                for c in range(1, COLS - 1):
                    if self.map[r][c] == 0:
                        # Check neighbors for Unknown
                        for nr, nc in [(r-1,c), (r+1,c), (r,c-1), (r,c+1)]:
                            if self.map[nr][nc] == -1:
                                frontiers.append((r,c))
                                break
                                
            if not frontiers:
                self.finished = True
                return

            # Simple Heuristic: Closest Frontier
            cr, cc = int(self.current_pose[1]//CELL_SIZE), int(self.current_pose[0]//CELL_SIZE)
            start = (cr, cc)
            
            # Sort frontiers by distance
            frontiers.sort(key=lambda p: abs(p[0]-start[0]) + abs(p[1]-start[1]))
            target = frontiers[0]
            
            self.path = self.astar(start, target)
        if self.path: return # Already have a plan
        
        # Identify Frontiers
        frontiers = []
        for r in range(1, ROWS - 1):
            for c in range(1, COLS - 1):
                if self.map[r][c] == 0:
                    # Check neighbors for Unknown
                    for nr, nc in [(r-1,c), (r+1,c), (r,c-1), (r,c+1)]:
                        if self.map[nr][nc] == -1:
                            frontiers.append((r,c))
                            break
                            
        if not frontiers:
            self.finished = True
            return

        # Simple Heuristic: Closest Frontier
        cr, cc = int(self.current_pose[1]//CELL_SIZE), int(self.current_pose[0]//CELL_SIZE)
        start = (cr, cc)
        
        # Sort frontiers by distance
        frontiers.sort(key=lambda p: abs(p[0]-start[0]) + abs(p[1]-start[1]))#len(self.astar(start, p)))#
        target = frontiers[0]
        
        self.path = self.astar(start, target)

    def astar(self, start, goal):
        # ... (Standard A* implementation) ...
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            
            r, c = current
            for nr, nc in [(r-1,c), (r+1,c), (r,c-1), (r,c+1)]:
                if 0 <= nr < ROWS and 0 <= nc < COLS:
                    # Treat Unknown (-1) as walkable for exploration optimism, 
                    # but prefer Free (0).
                    if self.map[nr][nc] != 1: 
                        tentative_g = g_score[current] + 1
                        if tentative_g < g_score.get((nr,nc), float('inf')):
                            came_from[(nr,nc)] = current
                            g_score[(nr,nc)] = tentative_g
                            f_score = tentative_g + (abs(nr-goal[0]) + abs(nc-goal[1]))
                            heapq.heappush(open_set, (f_score, (nr,nc)))
        return []

# --- MAZE GENERATOR (Helper) ---
def generate_maze():
    grid = [[1 for _ in range(COLS)] for _ in range(ROWS)]
    start = (1,1)
    grid[1][1] = 0
    grid[-1][-1] = 0 # Ensure end is free
    stack = [start]
    while stack:
        r, c = stack[-1]
        neighbors = []
        for dr, dc in [(-2,0), (2,0), (0,-2), (0,2)]:
            nr, nc = r+dr, c+dc
            if 1 <= nr < ROWS-1 and 1 <= nc < COLS-1:
                if grid[nr][nc] == 1: neighbors.append((nr,nc))
        if neighbors:
            nr, nc = random.choice(neighbors)
            grid[(r+nr)//2][(c+nc)//2] = 0
            grid[nr][nc] = 0
            stack.append((nr,nc))
        else:
            stack.pop()
    # Add loops
    for _ in range(20):
        r, c = random.randint(1, ROWS-2), random.randint(1, COLS-2)
        if grid[r][c] == 1 and grid[r-1][c]==0 and grid[r+1][c]==0: grid[r][c]=0
    return grid

# --- MAIN LOOP ---
def main():
    draw_pygame = True

    if draw_pygame:
        pygame.init()
        screen = pygame.display.set_mode((WIDTH, HEIGHT))
        clock = pygame.time.Clock()
    
    # 1. Init Simulation ("Gazebo")
    raw_maze = generate_maze()
    sim_node = PhysicsNode(raw_maze)
    
    # 2. Init Navigation ("ROS Stack")
    nav_node = NavigationNode((sim_node.x, sim_node.y))

    state = 0
    
    running = True
    while running:
        # --- ROS2 LOOP SIMULATION ---
        
        # 1. PHYSICS UPDATE (Hardware)
        sim_node.update_physics()
        
        # 2. SENSOR PUBLISH (Hardware -> Topic)
        scan_msg = sim_node.generate_scan()
        robot_pose = (sim_node.x, sim_node.y, sim_node.theta) # Mock TF
        
        # 3. NODE PROCESSING (Topic -> Brain)
        nav_node.scan_callback(scan_msg, robot_pose)

        if state == 0: # explore maze
            nav_node.plan_path()
        elif state == 1: # go back to start
            print("Planning path back to start")
            cr, cc = int(nav_node.current_pose[1]//CELL_SIZE), int(nav_node.current_pose[0]//CELL_SIZE)

            nav_node.path = nav_node.astar((cr, cc), (1,1)) # Clear path to trigger replan
            state = 2
        elif state == 3: # go to end
            print("Planning path to end")
            cr, cc = int(nav_node.current_pose[1]//CELL_SIZE), int(nav_node.current_pose[0]//CELL_SIZE)


            nav_node.path = nav_node.astar((cr, cc), (ROWS - 2, COLS - 2))#(ROWS - 1, COLS - 1)) # Clear path to trigger replan
            state = 4
        elif state == 2 or state == 4:
            if not nav_node.path:
                state += 1
        else:
            nav_node.finished = True
            
        cmd_vel = nav_node.compute_cmd_vel() # Generate Twist
        
        # 4. COMMAND SUBSCRIBE (Brain -> Hardware)
        sim_node.cmd_vel_callback(cmd_vel)

        if nav_node.finished:
            #print("Exploration Complete!")
            state += 1
            nav_node.finished = False
            nav_node.path = []

            if state > 5:
                break
            

        if not draw_pygame:
            continue

        # --- VISUALIZATION (RVIZ REPLACEMENT) ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
        screen.fill(COLOR_BG)
        
        # Draw "Real" Maze (Ghostly)
        # for r in range(ROWS):
        #     for c in range(COLS):
        #         if raw_maze[r][c] == 1:
        #             pygame.draw.rect(screen, (30,30,40), (c*CELL_SIZE, r*CELL_SIZE, CELL_SIZE, CELL_SIZE))

        # Draw "Mapped" Maze (Robot's Brain)
        for r in range(ROWS):
            for c in range(COLS):
                cell = nav_node.map[r][c]
                rect = (c*CELL_SIZE, r*CELL_SIZE, CELL_SIZE, CELL_SIZE)
                if cell == 0: pygame.draw.rect(screen, COLOR_FREE, rect)
                elif cell == 1: pygame.draw.rect(screen, COLOR_WALL, rect)
        
        # Draw A* Path
        if nav_node.path:
            points = [(p[1]*CELL_SIZE+CELL_SIZE/2, p[0]*CELL_SIZE+CELL_SIZE/2) for p in nav_node.path]
            if len(points) > 1: pygame.draw.lines(screen, COLOR_PATH, False, points, 2)

        # draw S for start and E for end
        font = pygame.font.SysFont(None, 24)
        start_text = font.render('S', True, COLOR_GOAL)
        end_text = font.render('E', True, COLOR_GOAL)
        screen.blit(start_text, (1*CELL_SIZE+CELL_SIZE/4, 1*CELL_SIZE+CELL_SIZE/4))
        screen.blit(end_text, ((COLS-2)*CELL_SIZE+CELL_SIZE/4, (ROWS-2)*CELL_SIZE+CELL_SIZE/4))

        # Draw Robot Body
        pygame.draw.circle(screen, COLOR_ROBOT, (int(sim_node.x), int(sim_node.y)), 8)
        
        # Draw Heading Arrow
        end_x = sim_node.x + math.cos(sim_node.theta) * 15
        end_y = sim_node.y + math.sin(sim_node.theta) * 15
        pygame.draw.line(screen, (255,0,0), (sim_node.x, sim_node.y), (end_x, end_y), 2)

        pygame.display.flip()
        clock.tick(FPS)

    nav_node.maze_to_file("test_map.json")

    pygame.quit()

if __name__ == "__main__":
    start = time()

    cells_x = 15
    cells_y = 15

    WIDTH = cells_x * CELL_SIZE
    HEIGHT = cells_y * CELL_SIZE
    COLS = WIDTH // CELL_SIZE
    ROWS = HEIGHT // CELL_SIZE

    runs = 1
    for i in range(runs):
        if (i+1) % 100 == 0:
            print(f"Run {i+1}/{runs} - Average steps so far: {total_steps/(i+1):.2f}")
        main()

    print(f"Average steps to explore {cells_x}x{cells_y} maze: {total_steps/runs:.2f}")