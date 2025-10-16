import os

import matplotlib.pyplot as plt
import numpy as np

MAP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '3-map/map.npy')


### START CODE HERE ###
# This code block is optional. You can define your utility function and class in this block if necessary.

import heapq

from scipy.ndimage import distance_transform_edt


class Node:
    """Node class for Improved A* algorithm"""
    def __init__(self, position, parent=None, direction=None):
        self.position = position
        self.parent = parent
        self.direction = direction  # Direction from parent to this node
        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic cost from current node to goal
        self.f = 0  # Total cost (g + h)

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.position == other.position


def heuristic(pos1, pos2):
    """Euclidean distance heuristic for 8-directional movement"""
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)


def is_valid(world_map, position):
    """Check if position is valid and traversable"""
    x, y = position
    rows, cols = world_map.shape
    if 0 <= x < rows and 0 <= y < cols:
        return world_map[x][y] == 0
    return False


def compute_distance_map(world_map):
    """Compute distance transform from obstacles"""
    # Create binary map where 0 is obstacle, 1 is free space
    free_space = (world_map == 0).astype(float)
    # Compute distance from each free cell to nearest obstacle
    distance_map = distance_transform_edt(free_space)
    return distance_map


def get_direction(from_pos, to_pos):
    """Get direction tuple from one position to another"""
    return (to_pos[0] - from_pos[0], to_pos[1] - from_pos[1])


def calculate_steering_cost(prev_direction, new_direction):
    """Calculate cost for changing direction (penalize turns)"""
    if prev_direction is None:
        return 0

    # If direction is the same, no steering cost
    if prev_direction == new_direction:
        return 0

    # Calculate angle change
    # For 8-directional movement, we can have different turn angles
    # Straight continuation: 0 cost
    # 45-degree turn: small cost
    # 90-degree turn: medium cost
    # 135-degree turn: high cost
    # 180-degree turn (reverse): very high cost

    dot_product = prev_direction[0] * new_direction[0] + prev_direction[1] * new_direction[1]

    # Normalize by direction magnitude
    prev_mag = np.sqrt(prev_direction[0]**2 + prev_direction[1]**2)
    new_mag = np.sqrt(new_direction[0]**2 + new_direction[1]**2)

    if prev_mag > 0 and new_mag > 0:
        cos_angle = dot_product / (prev_mag * new_mag)
        # cos_angle: 1 (same direction), 0 (90 degrees), -1 (opposite)
        # Convert to cost: 0 for same direction, higher for larger turns
        steering_cost = (1 - cos_angle) * 0.5  # Scale factor for steering penalty
        return steering_cost

    return 0


###  END CODE HERE  ###


def Improved_A_star(world_map, start_pos, goal_pos):
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

    # Compute distance map from obstacles
    distance_map = compute_distance_map(world_map)

    # Initialize start and goal nodes
    start_node = Node(start_pos, direction=None)
    start_node.g = 0
    start_node.h = heuristic(start_pos, goal_pos)
    start_node.f = start_node.g + start_node.h

    # Initialize open and closed lists
    open_list = []
    heapq.heappush(open_list, start_node)
    closed_set = set()

    # Dictionary to track best g-score for each position
    g_scores = {start_pos: 0}

    # Define 8-directional movements (including diagonals)
    movements = [
        (0, 1),    # right
        (0, -1),   # left
        (1, 0),    # down
        (-1, 0),   # up
        (1, 1),    # down-right (diagonal)
        (1, -1),   # down-left (diagonal)
        (-1, 1),   # up-right (diagonal)
        (-1, -1)   # up-left (diagonal)
    ]

    # Cost weights for different factors
    OBSTACLE_DISTANCE_WEIGHT = 2.0  # Weight for obstacle proximity penalty
    STEERING_WEIGHT = 5.0           # Weight for steering cost
    MIN_SAFE_DISTANCE = 3.0         # Minimum safe distance from obstacles

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

            # Calculate movement cost
            # Diagonal movements cost sqrt(2), orthogonal movements cost 1
            if abs(move[0]) + abs(move[1]) == 2:  # Diagonal
                movement_cost = np.sqrt(2)
            else:  # Orthogonal
                movement_cost = 1.0

            # Calculate obstacle distance penalty
            # Penalize paths that are too close to obstacles
            obstacle_distance = distance_map[neighbor_pos[0], neighbor_pos[1]]
            if obstacle_distance < MIN_SAFE_DISTANCE:
                # Add penalty for being close to obstacles
                obstacle_penalty = OBSTACLE_DISTANCE_WEIGHT * (MIN_SAFE_DISTANCE - obstacle_distance)
            else:
                obstacle_penalty = 0

            # Calculate steering cost
            new_direction = get_direction(current_node.position, neighbor_pos)
            steering_cost = calculate_steering_cost(current_node.direction, new_direction)
            steering_penalty = STEERING_WEIGHT * steering_cost

            # Create neighbor node
            neighbor_node = Node(neighbor_pos, current_node, new_direction)

            # Total g cost includes movement, obstacle proximity, and steering
            neighbor_node.g = current_node.g + movement_cost + obstacle_penalty + steering_penalty
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
    path = Improved_A_star(map, start_pos, goal_pos)

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
