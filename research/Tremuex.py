from MappingSimulation import *
from random import randint

random_direction = True
debug_output = False

def tremuex(maze):
    was_here = []
    correct_path = []
    for w in range(maze.width):
        was_here.append([False] * maze.height)
        correct_path.append([False] * maze.height)

    return _tremuex(maze, was_here, correct_path)

def _tremuex(maze, was_here, correct_path):
    x,y = maze.bot_position

    if maze.bot_position == maze.end: return True
    if was_here[x][y]: return False
    was_here[x][y] = True

    lidar_result = maze.scan_lidar_basic()

    if not random_direction:
        directions = ((0,2), (1, 3), (2, 0), (3, 1))
        for d in directions:
            if not maze.move(d[0]):
                continue
            if debug_output:
                print(d[0])
                maze.print_map()
                time.sleep(0.1)

            worked = _tremuex(maze, was_here, correct_path)
            
            if maze.is_done(): return True
            
            maze.move(d[1])

            if debug_output:
                print(d[1])
                maze.print_map()
                time.sleep(0.1)
            if worked:
                correct_path[x][y] = True
                return True
    else:
        directions = [(0,2), (1, 3), (2, 0), (3, 1)]
        for _ in range(4):
            i = randint(0, len(directions) - 1)
            d = directions[i]
            directions.pop(i)
            if not maze.move(d[0]):
                continue
            if debug_output:
                print(d[0])
                maze.print_map()
                time.sleep(0.1)
            worked = _tremuex(maze, was_here, correct_path)
            
            if maze.is_done(): return True
            
            maze.move(d[1])
            if debug_output:
                print(d[1])
                maze.print_map()
                time.sleep(0.1)
            if worked:
                correct_path[x][y] = True
                return True

    return False


if __name__ == "__main__":
    sim = load_maze('test.map')
    total_steps = []
    for i in range(5):
        tremuex(sim)
        total_steps.append(sim.total_steps)
        sim.reset()

    f = open('out.dat', 'w')
    f.write("\n".join(list(map(str, total_steps))))

    print("With random:")
    print(sum(total_steps) / len(total_steps))
    print("\nWithout random:")
    random_direction = False
    tremuex(sim)
    print(sim.total_steps)