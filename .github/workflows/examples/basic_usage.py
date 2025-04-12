# Contents of /hierarchical-quantum-pathfinding/hierarchical-quantum-pathfinding/examples/basic_usage.py

from hqp.core import hqp

def main():
    # Define a simple maze (0 = open space, 1 = wall)
    maze = [
        [0, 1, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 0, 0],
        [0, 0, 0, 1, 0]
    ]
    
    # Create an instance of the hqp class
    pathfinder = hqp(maze)
    
    # Define start and goal points
    start = (0, 0)
    goal = (4, 4)
    
    # Find the path
    path = pathfinder.find_path(start, goal)
    
    # Print the result
    if path:
        print("Path found:", path)
    else:
        print("No path found.")

if __name__ == "__main__":
    main()