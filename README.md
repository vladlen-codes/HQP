# Hierarchical Quantum Pathfinding Algorithm (HQP)

## Overview
The Hierarchical Quantum Pathfinding Algorithm (HQP) is an advanced pathfinding solution that combines hierarchical spatial decomposition with quantum-inspired parallel path exploration. It's designed for efficient navigation in large, complex environments with dynamic obstacles.

## Key Features
- **Hierarchical Region-Based Navigation**: Divides the environment into manageable regions for efficient pathfinding
- **Quantum Path Exploration**: Generates multiple potential paths in parallel
- **Path Caching**: Stores previously calculated paths for quick retrieval
- **Dynamic Obstacle Handling**: Adapts to obstacles that appear or disappear during runtime
- **Points Avoidance**: Supports designating specific areas to avoid during pathfinding

## Algorithm Components

### Region Management
- Automatically divides the environment into regions of configurable size
- Identifies connections between adjacent regions
- Maps points to their containing regions

### Pathfinding Approaches
- **Local A***: Finds paths within a single region
- **Region-Level Search**: Plans high-level paths across regions
- **Full A***: Performs traditional A* search when necessary

### Quantum Path Exploration
- Generates multiple candidate paths in parallel
- Creates path variations to explore diverse solutions
- Collapses multiple paths to select the optimal solution based on length and obstacle avoidance

### Cache System
- Caches paths for quick retrieval in similar scenarios
- Selectively invalidates cache entries when obstacles change

## Usage Example

```python
# Create a grid-based environment (0 = open space, 1 = wall)
maze = [
    [0, 0, 0, 0, 1, 0, 0, 0],
    [0, 1, 1, 0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0]
]

# Initialize the HQP pathfinder
pathfinder = hqp(maze, region_size=2, num_parallel_paths=3)

# Find a path from start to goal
start = (0, 0)
goal = (7, 4)
path = pathfinder.find_path(start, goal)

# Update dynamic obstacles
pathfinder.update_dynamic_obstacles([(3, 2), (4, 2)])

# Set points to avoid
pathfinder.set_points_to_avoid([(5, 3), (5, 2)])

