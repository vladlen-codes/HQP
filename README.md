# HQP
Hierarchical Quantum Pathfinding Algorithm (HQP), which finds optimal paths through a maze represented as a 2D grid (0s for navigable cells, 1s for walls). It is designed to handle both static walls and dynamic obstacles, with a focus on efficiency and adaptability.

Hierarchical Approach
The algorithm consists two-level planning strategy:
Region Division and Planning:
* The maze is divided into smaller regions, typically 4x4 cells, using the _build_regions method. Each region is identified by an ID, with information about its cells, center, and bounds stored for later use.
* Region-level planning is performed using _region_level_search, which applies an A* algorithm at the region level. This involves finding a sequence of regions that connect the start region (containing the start point) to the goal region (containing the goal point). The A* search uses region connections (precomputed by _identify_region_connections) and a heuristic based on the distance between region centers.
* This hierarchical structure reduces computational complexity by first planning at a coarser level, avoiding the need to search the entire maze at once.
Local Pathfinding:
* Once a region-level path is determined, detailed paths are found within and between regions using local A* searches (_local_astar and _full_astar). These methods implement the A* algorithm, considering static walls, dynamic obstacles, and points to avoid, ensuring optimal paths at the cell level.
* The _build_path_through_regions method constructs complete paths by chaining local paths through region connection points, ensuring continuity from start to goal.
This hierarchical approach is inspired by previous work on hierarchical pathfinding, such as Hierarchical Pathfinding and AI-Based Learning Approach in Strategy Game Design, which demonstrates improved efficiency in large graphs by dividing them into subgraphs.


Quantum-Inspired Exploration
The "quantum-inspired" aspect refers to the algorithm's method of exploring multiple potential paths in parallel, mimicking concepts from quantum mechanics, particularly superposition. This is implemented in the _explore_quantum_paths method:
* The method generates multiple candidate paths (defaulting to 3, controlled by num_parallel_paths) by varying connection points between regions and using local A* searches. For example, it may choose different entry and exit points for region transitions, introducing variability.
* If the start and goal are in different regions, it first plans a region-level path and then builds detailed paths through these regions. If fewer paths are generated than requested, it creates variations of existing paths using _create_path_variation, such as finding alternative routes around random segments.
* The term "parallel" here is metaphorical, not implying actual parallel processing. Instead, it means the algorithm logically considers multiple path options simultaneously, similar to how quantum superposition allows a system to be in multiple states before measurement.
This approach is inspired by research on quantum pathfinding, such as TransPath: Learning Heuristics For Grid-Based Pathfinding via Transformers, which explores parallel exploration in grid-based environments, though the implementation here is classical and does not use quantum computing.

The final path selection is handled by _collapse_quantum_paths, which evaluates all generated paths:
* Each path is scored based on its length (number of steps) and penalties for dynamic obstacles. The obstacle penalty is calculated as 100 * (1.0 - i/len(path)) for each obstacle encountered, where i is the index of the point in the path, ensuring higher penalties for obstacles earlier in the path.
* The path with the lowest total score (length + penalties) is selected, ensuring efficiency and safety.

Handling Dynamic Obstacles
Dynamic obstacles (temporary walls) are a key challenge in real-world pathfinding, such as in robotics or video games. The algorithm addresses this through:
* Caching Mechanism: Paths are cached based on start points, goal points, and dynamic obstacles, using _generate_cache_key to create unique keys. This improves performance by avoiding redundant calculations for the same scenarios.
* Cache Invalidation: When dynamic obstacles change, the update_dynamic_obstacles method identifies affected regions and calls invalidate_cache_region to remove cached paths involving those regions. This ensures that the algorithm adapts to environmental changes without recomputing unaffected paths.
* Path Reevaluation: During path exploration, dynamic obstacles are considered in both local A* searches and path scoring, ensuring that paths intersecting obstacles are penalized, encouraging avoidance.
This adaptability is crucial for applications like autonomous navigation, where obstacles may appear unexpectedly, and is supported by research on dynamic pathfinding, such as Learning heuristics for A*, which discusses adaptive strategies for changing environments.

Comparative Analysis
To understand the novelty of this algorithm, compare it with traditional A*:
* Traditional A*: Uses a single heuristic (e.g., Manhattan distance) to guide the search, exploring nodes in a priority queue until the goal is reached. It is optimal if the heuristic is admissible but can be slow in large mazes or with many obstacles.
* Hierarchical Quantum-Inspired: Divides the maze into regions, reducing the search space at the region level, and explores multiple paths in parallel, increasing the chance of finding optimal or alternative routes. It handles dynamic obstacles through caching and invalidation, making it more adaptable.

Approach	Method	Efficiency	Optimality	Dynamic Obstacle Handling
Traditional A*	Single heuristic search	Moderate	Guaranteed	Limited (recompute)
Hierarchical Quantum-Inspired	Hierarchical, multiple paths	High	Configurable	Adaptive (cache update)
This comparison highlights the algorithm's advantages in efficiency and adaptability, particularly for dynamic environments.

Benefits and Applications
The Hierarchical Quantum-Inspired Pathfinding Algorithm offers several benefits:
* Efficiency: The hierarchical structure reduces computational complexity, making it suitable for large mazes. By planning at the region level first, it avoids searching the entire grid, as supported by Hierarchical Pathfinding and AI-Based Learning Approach in Strategy Game Design.
* Adaptability: Handling dynamic obstacles through caching and invalidation ensures it can adapt to changing environments, crucial for robotics and video games.
* Robustness: Exploring multiple paths increases the likelihood of finding a valid path, especially when some routes are blocked by obstacles, mimicking the robustness seen in quantum-inspired search strategies like TransPath: Learning Heuristics For Grid-Based Pathfinding via Transformers.
* Applications: It is particularly useful in:
    * Robotics: Navigation in dynamic environments, such as warehouses with moving obstacles.
    * Video Games: Procedural level generation and AI opponent navigation, enhancing gameplay with adaptive paths.
    * Logistics: Route planning in changing traffic conditions, ensuring efficient delivery routes.
Challenges and Considerations
While the algorithm is innovative, there are challenges:
* Generalization: Ensuring the region size and number of parallel paths (num_parallel_paths) are optimal for different maze sizes and configurations may require tuning.
* Optimality: While it aims for optimality, the multiple path exploration may not always guarantee the shortest path if the heuristic is not admissible or if dynamic obstacles significantly alter the environment.
* Computational Cost: Generating multiple paths and evaluating them can be resource-intensive, though caching mitigates this for repeated queries.
Conclusion
The Hierarchical Quantum-Inspired Pathfinding Algorithm represents a sophisticated approach to pathfinding, combining hierarchical planning with parallel path exploration inspired by quantum mechanics. It is particularly effective for dynamic mazes, offering improved efficiency and adaptability compared to traditional methods like A*. This makes it a promising solution for applications in robotics, gaming, and logistics, with potential for further research into optimizing region sizes and path generation strategies.


