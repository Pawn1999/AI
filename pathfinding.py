import heapq

class Node:
    def __init__(self, position, parent=None, action=None):
        self.position = position
        self.parent = parent
        self.action = action
        self.cost = 0

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

    def __lt__(self, other):
        return self.cost < other.cost

def read_map(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        size = tuple(map(int, lines[0].split()))
        start = tuple(map(int, lines[1].split()))
        goal = tuple(map(int, lines[2].split()))
        grid = []
        for line in lines[3:]:
            row = []
            for char in line.strip():
                if char == 'X':
                    row.append(char)
                else:
                    row.append(int(char))
            grid.append(row)

    return size, start, goal, grid

def bfs(map_grid, start, goal):
    from collections import deque

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    start_node = Node(start)
    goal_node = Node(goal)
    frontier = deque([start_node])
    explored = set()
    
    while frontier:
        current_node = frontier.popleft()
        explored.add(current_node.position)

        if current_node.position == goal:
            return construct_path(current_node)
        
        for direction in directions:
            new_position = (current_node.position[0] + direction[0],
                            current_node.position[1] + direction[1])
            
            if (0 <= new_position[0] < len(map_grid)) and (0 <= new_position[1] < len(map_grid[0])):
                if map_grid[new_position[0]][new_position[1]] != 'X':
                    child = Node(new_position, current_node)
                    
                    if child.position not in explored and all(frontier_node.position != new_position for frontier_node in frontier):
                        frontier.append(child)
    return None

def construct_path(node):
    path = []
    while node.parent is not None:
        path.append(node.position)
        node = node.parent
    path.append(node.position)
    return path[::-1]

def update_map_with_path(map_grid, path):
    updated_map = [row[:] for row in map_grid]
    for position in path:
        updated_map[position[0]][position[1]] = '*'
    return updated_map

def manhattan_distance(start, goal):
    return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

def euclidean_distance(start, goal):
    return ((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2) ** 0.5

def ucs(map_grid, start, goal):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    frontier = []
    heapq.heappush(frontier, (0, Node(start)))
    explored = set()

    while frontier:
        current_cost, current_node = heapq.heappop(frontier)
        if current_node.position == goal:
            return construct_path(current_node)

        explored.add(current_node.position)

        for direction in directions:
            new_position = (current_node.position[0] + direction[0],
                            current_node.position[1] + direction[1])

            if (0 <= new_position[0] < len(map_grid)) and (0 <= new_position[1] < len(map_grid[0])):
                if map_grid[new_position[0]][new_position[1]] != 'X':
                    child_cost = current_cost + 1
                    child_node = Node(new_position, current_node)

                    if new_position not in explored and not any(child_node.position == node.position and child_cost >= node.cost for _, node in frontier):
                        heapq.heappush(frontier, (child_cost, child_node))

    return None

def a_star(map_grid, start, goal, heuristic=manhattan_distance):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    frontier = []
    heapq.heappush(frontier, (0 + heuristic(start, goal), Node(start), 0))
    explored = set()

    while frontier:
        current_total_cost, current_node, current_path_cost = heapq.heappop(frontier)
        if current_node.position == goal:
            return construct_path(current_node)

        explored.add(current_node.position)

        for direction in directions:
            new_position = (current_node.position[0] + direction[0],
                            current_node.position[1] + direction[1])

            if (0 <= new_position[0] < len(map_grid)) and (0 <= new_position[1] < len(map_grid[0])):
                if map_grid[new_position[0]][new_position[1]] != 'X':
                    step_cost = 1
                    child_cost = current_path_cost + step_cost
                    child_node = Node(new_position, current_node)

                    child_total_cost = child_cost + heuristic(new_position, goal)

                    if new_position not in explored and not any(child_node.position == node.position and child_total_cost >= (node.cost + heuristic(node.position, goal)) for _, node, _ in frontier):
                        heapq.heappush(frontier, (child_total_cost, child_node, child_cost))

    return None

if __name__ == "__main__":
    # Read the map from file
    map_file = "sample_map.txt"
    size, start, goal, map_grid = read_map(map_file)
    
    # Run BFS, UCS, and A* with Manhattan heuristic
    bfs_path = bfs(map_grid, start, goal)
    ucs_path = ucs(map_grid, start, goal)
    a_star_path = a_star(map_grid, start, goal, heuristic=manhattan_distance)

    # Print the paths if they exist
    print("BFS path:")
    if bfs_path:
        updated_map = update_map_with_path(map_grid, bfs_path)
        for row in updated_map:
            print(''.join(str(cell) for cell in row))
    else:
        print("null")

    print("\nUCS path:")
    if ucs_path:
        updated_map = update_map_with_path(map_grid, ucs_path)
        for row in updated_map:
            print(''.join(str(cell) for cell in row))
    else:
        print("null")

    print("\nA* Search path with Manhattan heuristic:")
    if a_star_path:
        updated_map = update_map_with_path(map_grid, a_star_path)
        for row in updated_map:
            print(''.join(str(cell) for cell in row))
    else:
        print("null")
