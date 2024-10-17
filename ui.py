import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import heapq
import random

# Create a simple warehouse-like environment (a 20x20 grid with obstacles)
def create_warehouse_map():
    warehouse_map = np.zeros((20, 20))
    warehouse_map[5:10, 5] = 1  # Vertical obstacle
    warehouse_map[12:15, 8:12] = 1  # Horizontal obstacle
    return warehouse_map

# Dijkstra's Algorithm
def dijkstra(warehouse_map, start, goal):
    rows, cols = warehouse_map.shape
    queue = [(0, start)]
    costs = {start: 0}
    parents = {start: None}
    visited = set()

    while queue:
        cost, current = heapq.heappop(queue)

        if current == goal:
            path = []
            while current:
                path.append(current)
                current = parents[current]
            return path[::-1]  # Reverse the path to get start to goal

        visited.add(current)

        for delta_row, delta_col in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = (current[0] + delta_row, current[1] + delta_col)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if warehouse_map[neighbor] == 0 and neighbor not in visited:
                    new_cost = cost + 1
                    if new_cost < costs.get(neighbor, float('inf')):
                        costs[neighbor] = new_cost
                        parents[neighbor] = current
                        heapq.heappush(queue, (new_cost, neighbor))

    return None  # No path found

# A* Algorithm
def astar(warehouse_map, start, goal):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    rows, cols = warehouse_map.shape
    queue = [(0, start)]
    costs = {start: 0}
    parents = {start: None}
    visited = set()

    while queue:
        _, current = heapq.heappop(queue)

        if current == goal:
            path = []
            while current:
                path.append(current)
                current = parents[current]
            return path[::-1]

        visited.add(current)

        for delta_row, delta_col in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = (current[0] + delta_row, current[1] + delta_col)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if warehouse_map[neighbor] == 0 and neighbor not in visited:
                    new_cost = costs[current] + 1
                    priority = new_cost + heuristic(neighbor, goal)
                    if new_cost < costs.get(neighbor, float('inf')):
                        costs[neighbor] = new_cost
                        parents[neighbor] = current
                        heapq.heappush(queue, (priority, neighbor))

    return None  # No path found

# RRT (Rapidly-exploring Random Tree) Algorithm
def rrt(warehouse_map, start, goal, max_iter=1000):
    rows, cols = warehouse_map.shape
    nodes = {start: None}
    for _ in range(max_iter):
        rand_node = (random.randint(0, rows - 1), random.randint(0, cols - 1))
        nearest_node = min(nodes, key=lambda n: abs(n[0] - rand_node[0]) + abs(n[1] - rand_node[1]))
        
        if warehouse_map[rand_node] == 0:
            nodes[rand_node] = nearest_node
            if abs(rand_node[0] - goal[0]) + abs(rand_node[1] - goal[1]) < 2:
                nodes[goal] = rand_node
                break

    path = []
    current = goal
    while current:
        path.append(current)
        current = nodes[current]
    return path[::-1] if goal in nodes else None

# Modified A* Algorithm (Simple A* variant with a weighted heuristic)
def modified_astar(warehouse_map, start, goal):
    def heuristic(a, b):
        return 1.5 * (abs(a[0] - b[0]) + abs(a[1] - b[1]))

    rows, cols = warehouse_map.shape
    queue = [(0, start)]
    costs = {start: 0}
    parents = {start: None}
    visited = set()

    while queue:
        _, current = heapq.heappop(queue)

        if current == goal:
            path = []
            while current:
                path.append(current)
                current = parents[current]
            return path[::-1]

        visited.add(current)

        for delta_row, delta_col in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = (current[0] + delta_row, current[1] + delta_col)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if warehouse_map[neighbor] == 0 and neighbor not in visited:
                    new_cost = costs[current] + 1
                    priority = new_cost + heuristic(neighbor, goal)
                    if new_cost < costs.get(neighbor, float('inf')):
                        costs[neighbor] = new_cost
                        parents[neighbor] = current
                        heapq.heappush(queue, (priority, neighbor))

    return None  # No path found

# UI for Path Planning Algorithms
class PathPlanningApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Path Planning Simulation")

        # Create a button to run all algorithms
        self.run_all_button = tk.Button(root, text="Run All Algorithms", command=self.run_all_algorithms)
        self.run_all_button.pack()

        # Create matplotlib figure for plotting
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack()

    # Plot the warehouse map
    def plot_warehouse(self, warehouse_map):
        self.ax.clear()
        self.ax.imshow(warehouse_map, cmap='gray_r')
        self.canvas.draw()

    # Plot the path generated by the algorithm
    def plot_path(self, path, color, label):
        if path:
            path = np.array(path)
            self.ax.plot(path[:, 1], path[:, 0], color, label=label)
        self.canvas.draw()

    def run_all_algorithms(self):
        warehouse_map = create_warehouse_map()
        start, goal = (0, 0), (19, 19)

        self.plot_warehouse(warehouse_map)

        try:
            # Run Dijkstra
            dijkstra_path = dijkstra(warehouse_map, start, goal)
            self.plot_path(dijkstra_path, 'r', "Dijkstra")

            # Run A*
            astar_path = astar(warehouse_map, start, goal)
            self.plot_path(astar_path, 'b', "A*")

            # Run RRT
            rrt_path = rrt(warehouse_map, start, goal)
            self.plot_path(rrt_path, 'g', "RRT")

            # Run Modified A*
            modified_astar_path = modified_astar(warehouse_map, start, goal)
            self.plot_path(modified_astar_path, 'y', "Modified A*")

            # Add legend to show which color represents which algorithm
            self.ax.legend()
            self.canvas.draw()

        except Exception as e:
            print(f"Error while running algorithms: {e}")

# Main application window
root = tk.Tk()
app = PathPlanningApp(root)
root.mainloop()
