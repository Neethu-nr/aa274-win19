import numpy as np
from itertools import permutations

def compute_optimal_order(unordered_list, robot_pose):
    clean_unordered_list = [np.array([pose[0], pose[1]]) for pose in unordered_list]
    print("robot_pose", robot_pose)
    print("clean_unordered_list", clean_unordered_list)
    ordered_list = [robot_pose]

    augmented_unordered_list = clean_unordered_list + [(np.array(robot_pose))]
    print("augmented_unordered_list", augmented_unordered_list)
    n = len(augmented_unordered_list) 
    distances = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            distances[i, j] = np.linalg.norm(augmented_unordered_list[i] - augmented_unordered_list[j], ord=1)
    possible_paths = list(set(permutations([i for i in range(n-1)])))
    best_path = min([(compute_distance(path, distances, n), path) for path in possible_paths])[1]
    for i in best_path:
        ordered_list.append(clean_unordered_list[i])
    ordered_list.append(robot_pose)
    return ordered_list

def compute_distance(path, distances, n):
    distance = 0
    distance += distances[n-1, path[0]]
    for i in range(len(path)-1):
        distance += distances[path[i], path[i+1]]
    distance += distances[path[-1], n-1]
    return distance

robot_pose = [0, 0]
unordered_list = [[2, 1],
                  [1, 3],
                  [1, 0],
                  [0, 2],
                  [2, 2]]
best_path = compute_optimal_order(unordered_list, robot_pose)
print("best_path", best_path)