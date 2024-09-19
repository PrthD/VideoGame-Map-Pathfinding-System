
# VideoGame Map Pathfinding System

Welcome to the **VideoGame-Map-Pathfinding-System** repository, a Python-based project that implements two classic pathfinding algorithms: **Dijkstra's Algorithm** and **A***, applied to grid-based video game maps. This project is ideal for those studying artificial intelligence, game development, or algorithmic problem-solving. It offers visual representations and performance analysis, making it a great tool for practical use in game AI.

## Features

- **Dijkstra's Algorithm**: An implementation that finds the shortest path on a uniform-cost grid map, considering both cardinal (N, S, E, W) and diagonal movements.
- **A* Search Algorithm**: A faster pathfinding algorithm that uses the **Octile Distance** heuristic for grid-based maps with diagonal movements.
- **Map Visualization**: Graphical output showcasing explored nodes and final paths for both algorithms, helping to visualize the pathfinding process.
- **Performance Comparison**: The project compares Dijkstra's and A* in terms of computational time and nodes expanded, visualized through performance plots.
- **Customizable Maps**: Use any grid-based map, including the provided dataset, to test various pathfinding scenarios.

## Project Structure

The repository is organized as follows:

- **src/**: Contains all Python source files that handle the algorithms and the map processing.
  - `main.py`: Runs the search algorithms and handles the experiments.
  - `algorithms.py`: Contains the implementations of Dijkstra's and A* algorithms.
  - `map.py`: Defines the map structure and utility functions for pathfinding.
  - `plot_results.py`: Provides functionality to generate visual performance plots comparing the algorithms.
  
- **plots/**: Stores the visual output comparing the performance of Dijkstra's and A* in terms of nodes expanded and runtime.

## Map and Problem Files

This project includes a `dao-map.zip` file containing video game map instances and a `test_problems_den009d.txt` file containing test instances (start and goal states) used to evaluate the performance of the algorithms.

### Steps:
1. **Extract the Map Files**:
   After cloning the repository, extract the `dao-map.zip` file into a directory:
   ```bash
   unzip dao-map.zip -d dao-map
   ```

2. **Using the Test Problem File**:
   The `test_problems_den009d.txt` file includes a series of start and goal states for testing the algorithms on the extracted map instances. Ensure the file is in the same directory as your Python files or provide the correct path when running the algorithms.

## Algorithms Overview

### Dijkstra’s Algorithm
This algorithm explores all possible paths from the start point until the goal is reached, ensuring the shortest path is found. It's implemented to handle uniform costs for cardinal and diagonal moves on grid-based maps.

### A* Search Algorithm
A* is a more efficient search algorithm that leverages the **Octile Distance** heuristic to guide the search. It prioritizes diagonal moves where possible, reducing the total number of expanded nodes and improving runtime over Dijkstra’s algorithm.

## How to Run

1. Install Python 3 if not already installed.
2. Set up a virtual environment (optional but recommended):
   ```bash
   virtualenv -p python3 venv
   source venv/bin/activate
   ```
3. Install the required libraries:
   ```bash
   pip install -r requirements.txt
   ```
4. Run the project:
   ```bash
   python3 main.py
   ```
   For additional visualization and comparison plots, run:
   ```bash
   python3 main.py --plots
   ```

## Visualization

Upon running the algorithms, the **plot_map** function is used to generate visualizations of the explored nodes. The white areas indicate traversable regions, black areas represent walls, and gray regions show explored nodes, with the final path highlighted.

## Performance Comparison

Scatter plots are generated that compare the two algorithms:
- **nodes_expanded.png**: Compares the number of nodes expanded by each algorithm.
- **running_time.png**: Compares the runtime of each algorithm.

These plots can be found in the `plots/` directory. The file `plot_results.py` is responsible for creating these visual comparisons.
