import heapq
import math
import sys


class Map:
    def __init__(self, rows, cols, start_pos, end_pos, grid):
        self.rows = rows
        self.cols = cols
        self.start = (start_pos[0] - 1, start_pos[1] - 1)
        self.end = (end_pos[0] - 1, end_pos[1] - 1)
        self.grid = grid

    def is_valid(self, i, j):
        return 0 <= i < self.rows and 0 <= j < self.cols

    def is_obstacle(self, i, j):
        return self.grid[i][j] == 'X'

    def get_elevation(self, i, j):
        return int(self.grid[i][j])

    def get_neighbors(self, i, j):
        neighbors = []
        deltas = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for di, dj in deltas:
            ni, nj = i + di, j + dj
            if self.is_valid(ni, nj) and not self.is_obstacle(ni, nj):
                neighbors.append((ni, nj))
        return neighbors


class Node:
    def __init__(self, position, parent=None, cost=0, heuristic=0):
        self.position = position
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)


class Pathfinder:
    def __init__(self, search_algorithm, heuristic=None):
        self.search_algorithm = search_algorithm
        self.heuristic = heuristic

    def bfs(self, start, end, map):
        fringe = [Node(start)]
        closed = set()

        while fringe:
            node = fringe.pop(0)
            if node.position == end:
                return self.construct_path(node)
            closed.add(node.position)
            for neighbor in map.get_neighbors(*node.position):
                if neighbor not in closed:
                    fringe.append(Node(neighbor, node))

    def ucs(self, start, end, map):
        fringe = [(0, Node(start))]
        closed = set()

        while fringe:
            cost, node = heapq.heappop(fringe)
            if node.position == end:
                return self.construct_path(node)
            closed.add(node.position)
            for neighbor in map.get_neighbors(*node.position):
                new_cost = node.cost + map.get_elevation(*neighbor) + 1
                if neighbor not in closed:
                    heapq.heappush(fringe, (new_cost, Node(neighbor, node, new_cost)))

    def euclidean_distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def manhattan_distance(self, pos1, pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def a_star(self, start, end, map):
        fringe = [Node(start)]
        closed = set()

        while fringe:
            node = heapq.heappop(fringe)
            if node.position == end:
                return self.construct_path(node)
            closed.add(node.position)
            for neighbor in map.get_neighbors(*node.position):
                if neighbor not in closed:
                    cost = node.cost + map.get_elevation(*neighbor) + 1
                    if self.heuristic == 'euclidean':
                        heuristic = self.euclidean_distance(neighbor, end)
                    else:
                        heuristic = self.manhattan_distance(neighbor, end)
                    heapq.heappush(fringe, (cost + heuristic, Node(neighbor, node, cost, heuristic)))

    def construct_path(self, node):
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return path[::-1]

    def find_path(self, map):
        start = map.start
        end = map.end
        if self.search_algorithm == 'bfs':
            return self.bfs(start, end, map)
        elif self.search_algorithm == 'ucs':
            return self.ucs(start, end, map)
        elif self.search_algorithm == 'astar':
            return self.a_star(start, end, map)


if __name__ == "__main__":
    map_input = sys.stdin.readlines()
    map_info = map_input[0].strip().split()
    rows, cols = int(map_info[0]), int(map_info[1])
    start_pos = tuple(map(int, map_input[1].strip().split()))
    end_pos = tuple(map(int, map_input[2].strip().split()))
    grid = [line.strip().split() for line in map_input[3:]]

    map = Map(rows, cols, start_pos, end_pos, grid)

    algorithm = input().strip()
    heuristic = input().strip() if algorithm == 'astar' else None

    search_algorithm = Pathfinder(algorithm, heuristic)

    path = search_algorithm.find_path(map)

    for i in range(map.rows):
        for j in range(map.cols):
            if (i, j) == map.start:
                print('1', end=' ')
            elif (i, j) == map.end:
                print('10', end=' ')
            elif (i, j) in path:
                print('*', end=' ')
            else:
                print(map.grid[i][j], end=' ')
        print()
