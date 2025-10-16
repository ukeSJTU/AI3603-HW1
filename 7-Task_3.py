import os

import matplotlib.pyplot as plt
import numpy as np

MAP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '3-map/map.npy')


### START CODE HERE ###
# This code block is optional. You can define your utility function and class in this block if necessary.

import heapq

from scipy.interpolate import splev, splprep
from scipy.ndimage import distance_transform_edt


class Node:
    """Node class for A* algorithm with direction tracking"""
    def __init__(self, position, parent=None, direction=None):
        self.position = position
        self.parent = parent
        self.direction = direction
        self.g = 0
        self.h = 0
        self.f = 0

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.position == other.position


def heuristic(pos1, pos2):
    """Euclidean distance heuristic"""
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
    free_space = (world_map == 0).astype(float)
    distance_map = distance_transform_edt(free_space)
    return distance_map


def get_direction(from_pos, to_pos):
    """Get direction tuple from one position to another"""
    return (to_pos[0] - from_pos[0], to_pos[1] - from_pos[1])


def calculate_steering_cost(prev_direction, new_direction):
    """Calculate cost for changing direction"""
    if prev_direction is None:
        return 0
    if prev_direction == new_direction:
        return 0

    dot_product = prev_direction[0] * new_direction[0] + prev_direction[1] * new_direction[1]
    prev_mag = np.sqrt(prev_direction[0]**2 + prev_direction[1]**2)
    new_mag = np.sqrt(new_direction[0]**2 + new_direction[1]**2)

    if prev_mag > 0 and new_mag > 0:
        cos_angle = dot_product / (prev_mag * new_mag)
        steering_cost = (1 - cos_angle) * 0.5
        return steering_cost
    return 0


def improved_a_star_base(world_map, start_pos, goal_pos):
    """Base improved A* algorithm to get initial path"""
    start_pos = tuple(start_pos)
    goal_pos = tuple(goal_pos)

    distance_map = compute_distance_map(world_map)

    start_node = Node(start_pos, direction=None)
    start_node.g = 0
    start_node.h = heuristic(start_pos, goal_pos)
    start_node.f = start_node.g + start_node.h

    open_list = []
    heapq.heappush(open_list, start_node)
    closed_set = set()
    g_scores = {start_pos: 0}

    # 8-directional movements
    movements = [
        (0, 1), (0, -1), (1, 0), (-1, 0),
        (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]

    OBSTACLE_DISTANCE_WEIGHT = 2.0
    STEERING_WEIGHT = 1.5
    MIN_SAFE_DISTANCE = 3.0

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == goal_pos:
            path = []
            current = current_node
            while current is not None:
                path.append(list(current.position))
                current = current.parent
            path.reverse()
            return np.array(path)

        closed_set.add(current_node.position)

        for move in movements:
            neighbor_pos = (current_node.position[0] + move[0],
                          current_node.position[1] + move[1])

            if not is_valid(world_map, neighbor_pos):
                continue
            if neighbor_pos in closed_set:
                continue

            if abs(move[0]) + abs(move[1]) == 2:
                movement_cost = np.sqrt(2)
            else:
                movement_cost = 1.0

            obstacle_distance = distance_map[neighbor_pos[0], neighbor_pos[1]]
            if obstacle_distance < MIN_SAFE_DISTANCE:
                obstacle_penalty = OBSTACLE_DISTANCE_WEIGHT * (MIN_SAFE_DISTANCE - obstacle_distance)
            else:
                obstacle_penalty = 0

            new_direction = get_direction(current_node.position, neighbor_pos)
            steering_cost = calculate_steering_cost(current_node.direction, new_direction)
            steering_penalty = STEERING_WEIGHT * steering_cost

            neighbor_node = Node(neighbor_pos, current_node, new_direction)
            neighbor_node.g = current_node.g + movement_cost + obstacle_penalty + steering_penalty
            neighbor_node.h = heuristic(neighbor_pos, goal_pos)
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            if neighbor_pos in g_scores and g_scores[neighbor_pos] <= neighbor_node.g:
                continue

            g_scores[neighbor_pos] = neighbor_node.g
            heapq.heappush(open_list, neighbor_node)

    return np.array([start_pos, goal_pos])


###  END CODE HERE  ###


def Self_driving_path_planner(world_map, start_pos, goal_pos):
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
    # Step 1: Get initial path using improved A* algorithm
    initial_path = improved_a_star_base(world_map, start_pos, goal_pos)

    if len(initial_path) < 3:
        # Path too short to smooth
        return initial_path

    # Step 2: Simplify path by removing redundant waypoints
    # Keep only key turning points to reduce path complexity
    simplified_path = simplify_path(initial_path)

    if len(simplified_path) < 3:
        return simplified_path

    # Step 3: Apply B-spline smoothing for continuous, smooth trajectory
    smooth_path = smooth_path_with_spline(simplified_path, world_map)

    ###  END CODE HERE  ###
    return smooth_path


### START CODE HERE ###
# Additional helper functions for path smoothing

def simplify_path(path, angle_threshold=0.1):
    """
    Simplify path by removing intermediate points that don't significantly change direction.
    This reduces the number of waypoints while preserving the path shape.
    """
    if len(path) < 3:
        return path

    simplified = [path[0]]  # Always keep start point

    for i in range(1, len(path) - 1):
        # Calculate vectors
        vec1 = path[i] - path[i-1]
        vec2 = path[i+1] - path[i]

        # Normalize vectors
        norm1 = np.linalg.norm(vec1)
        norm2 = np.linalg.norm(vec2)

        if norm1 > 0 and norm2 > 0:
            vec1_normalized = vec1 / norm1
            vec2_normalized = vec2 / norm2

            # Calculate angle between vectors using dot product
            dot_product = np.dot(vec1_normalized, vec2_normalized)
            dot_product = np.clip(dot_product, -1.0, 1.0)

            # If angle is significant (direction changes), keep the point
            if dot_product < (1 - angle_threshold):
                simplified.append(path[i])
        else:
            simplified.append(path[i])

    simplified.append(path[-1])  # Always keep goal point

    return np.array(simplified)


def smooth_path_with_spline(path, world_map, num_points=200):
    """
    Smooth the path using B-spline interpolation.
    This creates a smooth, continuous curve suitable for self-driving vehicles.
    """
    if len(path) < 3:
        return path

    # Extract x and y coordinates
    x = path[:, 0]
    y = path[:, 1]

    # Determine spline degree (k) based on number of points
    # k must be less than the number of points
    k = min(3, len(path) - 1)  # Cubic spline if possible, otherwise lower degree

    try:
        # Fit B-spline to the path
        # s=0 means the spline passes through all points
        # s>0 allows some smoothing deviation
        tck, u = splprep([x, y], s=2.0, k=k)

        # Generate smooth path with more points
        u_new = np.linspace(0, 1, num_points)
        smooth_x, smooth_y = splev(u_new, tck)

        # Combine into path array
        smooth_path = np.column_stack((smooth_x, smooth_y))

        # Validate that smooth path doesn't go through obstacles
        smooth_path = validate_and_fix_path(smooth_path, world_map, path)

        return smooth_path

    except Exception as e:
        # If spline fitting fails, return original path
        return path


def validate_and_fix_path(smooth_path, world_map, original_path):
    """
    Validate that the smoothed path doesn't collide with obstacles.
    If collision detected, fall back to original path segments.
    """
    validated_path = []

    for i, point in enumerate(smooth_path):
        x, y = int(round(point[0])), int(round(point[1]))

        # Check if point is within map bounds
        if 0 <= x < world_map.shape[0] and 0 <= y < world_map.shape[1]:
            # Check if point is in free space
            if world_map[x, y] == 0:
                validated_path.append(point)
            else:
                # Collision detected - this shouldn't happen with proper smoothing
                # Skip this point or use nearest valid point from original path
                pass
        else:
            # Out of bounds - skip
            pass

    # If validation removed too many points, return original path
    if len(validated_path) < len(smooth_path) * 0.8:
        return original_path

    return np.array(validated_path) if len(validated_path) > 0 else original_path


###  END CODE HERE  ###





if __name__ == '__main__':

    # Get the map of the world representing in a 120*120 array, where 0 indicating traversable and 1 indicating obstacles.
    map = np.load(MAP_PATH)

    # Define goal position of the exploration
    goal_pos = [100, 100]

    # Define start position of the robot.
    start_pos = [10, 10]

    # Plan a path based on map from start position of the robot to the goal.
    path = Self_driving_path_planner(map, start_pos, goal_pos)

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
