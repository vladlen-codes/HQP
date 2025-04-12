# Hierarchical Quantum Pathfinding

Hierarchical Quantum Pathfinding (HQP) is a Python package designed for efficient pathfinding in 2D mazes. It utilizes a hierarchical approach to divide the maze into regions, allowing for faster navigation and the ability to handle dynamic obstacles. This package is suitable for applications in robotics, game development, and any scenario requiring efficient pathfinding algorithms.

## Features

- **Hierarchical Pathfinding**: Divides the maze into manageable regions for efficient searching.
- **Dynamic Obstacle Handling**: Updates paths in real-time as obstacles change.
- **Path Caching**: Caches previously computed paths for faster retrieval.
- **Multiple Path Exploration**: Generates multiple potential paths to choose from.

## Installation

You can install the package using pip:

```bash
pip install hierarchical-quantum-pathfinding
```

## Usage

Here is a basic example of how to use the `hqp` class for pathfinding in a maze:

```python
from hqp import hqp

# Define a maze where 0 represents open space and 1 represents walls
maze = [
    [0, 1, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0],
    [0, 0, 0, 1, 0]
]

# Initialize the pathfinding algorithm
pathfinder = hqp(maze)

# Find a path from start to goal
start = (0, 0)
goal = (4, 4)
path = pathfinder.find_path(start, goal)

print("Path found:", path)
```

## Contributing

Contributions are welcome! Please feel free to submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.