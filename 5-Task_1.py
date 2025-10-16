import os

import matplotlib.pyplot as plt
import numpy as np

MAP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '3-map/map.npy')


### START CODE HERE ###
# This code block is optional. You can define your utility function and class in this block if necessary.

import heapq


class Node:
    """Node class for A* algorithm"""
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic cost from current node to goal
        self.f = 0  # Total cost (g + h)

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.position == other.position

def heuristic(pos1, pos2):
    """Manhattan distance heuristic"""
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def is_valid(world_map, position):
    """Check if position is valid and traversable"""
    x, y = position
    rows, cols = world_map.shape
    if 0 <= x < rows and 0 <= y < cols:
        return world_map[x][y] == 0
    return False

###  END CODE HERE  ###


def A_star(world_map, start_pos, goal_pos):
    """
    Given map of the world, start position of the robot and the position of the goal, 
    plan a path from start position to the goal using A* algorithm.

    Arguments:
    world_map -- A 120*120 array indicating current map, where 0 indicating traversable and 1 indicating obstacles.
    start_pos -- A 2D vector indicating the current position of the robot.
    goal_pos -- A 2D vector indicating the position of the goal.

    Return:
    path -- A N*2 array representing the planned path by A* algorithm.
    """

    ### START CODE HERE ###
    # Convert positions to tuples for hashing
    start_pos = tuple(start_pos)
    goal_pos = tuple(goal_pos)

    # Initialize start and goal nodes
    start_node = Node(start_pos)
    start_node.g = 0
    start_node.h = heuristic(start_pos, goal_pos)
    start_node.f = start_node.g + start_node.h

    goal_node = Node(goal_pos)

    # Initialize open and closed lists
    open_list = []
    heapq.heappush(open_list, start_node)
    closed_set = set()

    # Dictionary to track best g-score for each position
    g_scores = {start_pos: 0}

    # Define 4-directional movements (up, down, left, right)
    movements = [
        (0, 1),   # right
        (0, -1),  # left
        (1, 0),   # down
        (-1, 0)   # up
    ]

    while open_list:
        # Get node with lowest f score
        current_node = heapq.heappop(open_list)

        # Check if we reached the goal
        if current_node.position == goal_pos:
            # Reconstruct path
            path = []
            current = current_node
            while current is not None:
                path.append(list(current.position))
                current = current.parent
            path.reverse()
            return np.array(path)

        # Add current node to closed set
        closed_set.add(current_node.position)

        # Explore neighbors
        for move in movements:
            neighbor_pos = (current_node.position[0] + move[0],
                          current_node.position[1] + move[1])

            # Check if neighbor is valid
            if not is_valid(world_map, neighbor_pos):
                continue

            # Skip if already in closed set
            if neighbor_pos in closed_set:
                continue

            # Calculate g, h, and f scores
            neighbor_node = Node(neighbor_pos, current_node)
            neighbor_node.g = current_node.g + 1  # Cost is 1 for each step
            neighbor_node.h = heuristic(neighbor_pos, goal_pos)
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            # Check if this path to neighbor is better
            if neighbor_pos in g_scores and g_scores[neighbor_pos] <= neighbor_node.g:
                continue

            # Add neighbor to open list
            g_scores[neighbor_pos] = neighbor_node.g
            heapq.heappush(open_list, neighbor_node)

    # No path found, return empty path
    path = np.array([start_pos, goal_pos])
    ###  END CODE HERE  ###
    return path





if __name__ == '__main__':

    # Get the map of the world representing in a 120*120 array, where 0 indicating traversable and 1 indicating obstacles.
    map = np.load(MAP_PATH)

    # Define goal position of the exploration
    goal_pos = [100, 100]

    # Define start position of the robot.
    start_pos = [10, 10]

    # Plan a path based on map from start position of the robot to the goal.
    path = A_star(map, start_pos, goal_pos)

    # Visualize the map and path.
    obstacles_x, obstacles_y = [], []
    for i in range(120):
        for j in range(120):
            if map[i][j] == 1:
                obstacles_x.append(i)
                obstacles_y.append(j)

    path_x, path_y = [], []
    for path_node in path:
        path_x.append(path_node[0])
        path_y.append(path_node[1])

    plt.plot(path_x, path_y, "-r")
    plt.plot(start_pos[0], start_pos[1], "xr")
    plt.plot(goal_pos[0], goal_pos[1], "xb")
    plt.plot(obstacles_x, obstacles_y, ".k")
    plt.grid(True)
    plt.axis("equal")
    plt.show()