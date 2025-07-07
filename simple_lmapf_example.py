#!/usr/bin/env python3
import pypibt
import random

# Setup
random.seed(42)
map_matrix = [
    [1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 1], 
    [1, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1]
]

starts = [(1, 1), (1, 2), (2, 1), (3, 4), (4, 4)]
goal_pools = [
    [(1, 1), (2, 2), (4, 1), (2, 4)],
    [(1, 2), (3, 1), (4, 2), (1, 4)],
    [(2, 1), (4, 4), (3, 2), (1, 3)],
    [(3, 4), (2, 3), (4, 3), (1, 1)],
    [(4, 1), (1, 4), (3, 3), (2, 1)]
]

# Random weights
edge_weights = [[1.0 for _ in range(5)] for _ in range(36)]

# Initialize
solver = pypibt.PIBT()
current_goals = [pool[0] for pool in goal_pools]
goal_indices = [0] * 5
goals_completed = [0] * 5

solver.initialize(map_matrix, starts, current_goals, True, edge_weights, False, 42)

# Run simulation
for step in range(100):
    positions = solver.step()
    if not positions: break
    
    # Check goals and update
    goals_changed = False
    new_goals = []
    for i in range(5):
        if positions[i] == current_goals[i]:
            goals_completed[i] += 1
            goal_indices[i] = (goal_indices[i] + 1) % len(goal_pools[i])
            current_goals[i] = goal_pools[i][goal_indices[i]]
            goals_changed = True
            new_goals.append((i, current_goals[i][0], current_goals[i][1]))
    
    if goals_changed:
        solver.update_goals(new_goals)

print(f"Final: {sum(goals_completed)} total goals, {[goals_completed[i] for i in range(5)]}") 