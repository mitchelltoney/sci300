from data import *
import numpy as np
from queue import deque
import matplotlib.pyplot as plt

#search algorithm
def bfs_search(grid, start, goal):
    grid = np.array(grid)
    neighbors = [(0,1), (1,0), (0,-1), (-1,0)] # movement options
    frontier = deque()
    frontier.append(start)
    
    came_from = {}
    came_from[start] = None
    while frontier:
        current = frontier.popleft()
        
        if current == goal:
            break
        
        for dx, dy in neighbors:
            next_cell = (current[0] + dx, current[1] + dy)
            if 0 <= next_cell[0] < grid.shape[0] and 0 <= next_cell[1] < grid.shape[1]:
                if grid[next_cell[0], next_cell[1]] == 0 or next_cell == goal:
                    if next_cell not in came_from:
                        frontier.append(next_cell)
                        came_from[next_cell] = current
    current = goal
    path = []
    while current != start: 
        if current in came_from:
            path.append(current)
            current = came_from[current]
        else:
            return []
    path.append(start)
    path.reverse()
    
    return path

results_bfs = {}
# change grids to find paths here
for i in range(1, 10):
    try:
        grid = eval(f'grid{i}')
        start = eval(f'start{i}')
        goal = eval(f'end{i}')
        path = bfs_search(grid, start, goal)
        results_bfs[f'grid{i}'] = path
    except NameError:
        break 

def visualize_path(grid, start, goal):
    path = bfs_search(grid, start, goal)
    grid_array = np.array(grid)
    plt.figure(figsize=(8, 8))
    ax = plt.gca()
    for y in range(grid_array.shape[0]):
        for x in range(grid_array.shape[1]):
            if grid_array[y, x] == 1:
                ax.fill([x, x, x+1, x+1], [y, y+1, y+1, y], "black")
            elif (y, x) in path:
                ax.fill([x, x, x+1, x+1], [y, y+1, y+1, y], "yellow")
    ax.fill([start[1], start[1], start[1]+1, start[1]+1], [start[0], start[0]+1, start[0]+1, start[0]], "green")  # Start
    ax.fill([goal[1], goal[1], goal[1]+1, goal[1]+1], [goal[0], goal[0]+1, goal[0]+1, goal[0]], "red")  # Goal
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    ax.set_xticks(np.arange(0, 11, 1))
    ax.set_yticks(np.arange(0, 11, 1))
    ax.set_xticklabels(np.arange(0, 11, 1))
    ax.set_yticklabels(np.arange(0, 11, 1))
    plt.grid(True)
    plt.title("Path Visualization")
    plt.show()

# output
for grid_name, path in results_bfs.items():
    print(f'{grid_name}: {path}')
visualize_path(grid1, start1, end1)