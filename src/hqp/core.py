import numpy as np
import heapq
from collections import defaultdict, deque
import hashlib
import json

class hqp:
    def __init__(self, maze, region_size=4, num_parallel_paths=3):
        self.maze = np.array(maze)
        self.height, self.width = self.maze.shape
        self.region_size = region_size
        self.num_parallel_paths = num_parallel_paths
        self.dynamic_obstacles = []
        self.points_to_avoid = []
        
        self.regions = {}
        self.region_connections = {}
        self.path_cache = {}
        
        self._build_regions()
        self._identify_region_connections()
    
    def _build_regions(self):
        region_id = 0
        for y in range(0, self.height, self.region_size):
            for x in range(0, self.width, self.region_size):
                x_end = min(x + self.region_size, self.width)
                y_end = min(y + self.region_size, self.height)
                
                cells = []
                for cy in range(y, y_end):
                    for cx in range(x, x_end):
                        if 0 <= cy < self.height and 0 <= cx < self.width and self.maze[cy, cx] == 0:
                            cells.append((cx, cy))
                
                if cells:
                    center_x = sum(c[0] for c in cells) / len(cells)
                    center_y = sum(c[1] for c in cells) / len(cells)
                    
                    self.regions[region_id] = {
                        'cells': cells,
                        'center': (center_x, center_y),
                        'bounds': (x, y, x_end, y_end)
                    }
                    region_id += 1
    
    def _identify_region_connections(self):
        for r_id, region in self.regions.items():
            self.region_connections[r_id] = []
            x_start, y_start, x_end, y_end = region['bounds']
            
            for other_id, other_region in self.regions.items():
                if r_id == other_id:
                    continue
                    
                ox_start, oy_start, ox_end, oy_end = other_region['bounds']
                
                if (x_start <= ox_end and x_end >= ox_start and 
                    (y_end == oy_start or y_start == oy_end)):
                    connection_points = []
                    for x in range(max(x_start, ox_start), min(x_end, ox_end)):
                        if oy_start == y_end:
                            if ((x, y_end-1) in region['cells'] and 
                                (x, oy_start) in other_region['cells']):
                                connection_points.append(((x, y_end-1), (x, oy_start)))
                        else:
                            if ((x, y_start) in region['cells'] and 
                                (x, oy_end-1) in other_region['cells']):
                                connection_points.append(((x, y_start), (x, oy_end-1)))
                    
                    if connection_points:
                        self.region_connections[r_id].append({
                            'region': other_id,
                            'connection_points': connection_points
                        })
                
                elif (y_start <= oy_end and y_end >= oy_start and 
                      (x_end == ox_start or x_start == ox_end)):
                    connection_points = []
                    for y in range(max(y_start, oy_start), min(y_end, oy_end)):
                        if ox_start == x_end:
                            if ((x_end-1, y) in region['cells'] and 
                                (ox_start, y) in other_region['cells']):
                                connection_points.append(((x_end-1, y), (ox_start, y)))
                        else:
                            if ((x_start, y) in region['cells'] and 
                                (ox_end-1, y) in other_region['cells']):
                                connection_points.append(((x_start, y), (ox_end-1, y)))
                    
                    if connection_points:
                        self.region_connections[r_id].append({
                            'region': other_id,
                            'connection_points': connection_points
                        })
    
    def _get_region_for_point(self, point):
        x, y = point
        for r_id, region in self.regions.items():
            if (x, y) in region['cells']:
                return r_id
        return None
    
    def _region_level_search(self, start_region, goal_region):
        if start_region == goal_region:
            return [start_region]
            
        open_set = [(0, start_region, [])]
        closed_set = set()
        g_scores = {start_region: 0}
        
        while open_set:
            _, current, path = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
                
            current_path = path + [current]
            
            if current == goal_region:
                return current_path
                
            closed_set.add(current)
            
            for connection in self.region_connections.get(current, []):
                neighbor = connection['region']
                
                if neighbor in closed_set:
                    continue
                    
                temp_g = g_scores[current] + self._distance(
                    self.regions[current]['center'], 
                    self.regions[neighbor]['center']
                )
                
                if neighbor not in g_scores or temp_g < g_scores[neighbor]:
                    g_scores[neighbor] = temp_g
                    f_score = temp_g + self._distance(
                        self.regions[neighbor]['center'], 
                        self.regions[goal_region]['center']
                    )
                    heapq.heappush(open_set, (f_score, neighbor, current_path))
        
        return None
    
    def _local_astar(self, start, goal, region_id):
        if start == goal:
            return [start]
            
        region = self.regions[region_id]
        
        open_set = [(0, start, [])]
        closed_set = set()
        g_scores = {start: 0}
        
        while open_set:
            _, current, path = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
                
            current_path = path + [current]
            
            if current == goal:
                return current_path
                
            closed_set.add(current)
            
            x, y = current
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nx, ny = x + dx, y + dy
                neighbor = (nx, ny)
                
                if (neighbor not in region['cells'] or 
                    neighbor in self.dynamic_obstacles or
                    neighbor in self.points_to_avoid):
                    continue
                    
                temp_g = g_scores[current] + 1
                
                if neighbor not in g_scores or temp_g < g_scores[neighbor]:
                    g_scores[neighbor] = temp_g
                    f_score = temp_g + self._distance(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor, current_path))
        
        return None
    
    def _full_astar(self, start, goal):
        if start == goal:
            return [start]
            
        open_set = [(0, start, [])]
        closed_set = set()
        g_scores = {start: 0}
        
        while open_set:
            _, current, path = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
                
            current_path = path + [current]
            
            if current == goal:
                return current_path
                
            closed_set.add(current)
            
            x, y = current
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nx, ny = x + dx, y + dy
                neighbor = (nx, ny)
                
                if (not (0 <= nx < self.width and 0 <= ny < self.height) or
                    self.maze[ny, nx] == 1 or
                    neighbor in self.dynamic_obstacles or
                    neighbor in self.points_to_avoid):
                    continue
                    
                temp_g = g_scores[current] + 1
                
                if neighbor not in g_scores or temp_g < g_scores[neighbor]:
                    g_scores[neighbor] = temp_g
                    f_score = temp_g + self._distance(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor, current_path))
        
        return None
    
    def _build_path_through_regions(self, region_path, start, goal):
        if len(region_path) <= 1:
            return self._local_astar(start, goal, region_path[0])
            
        full_path = []
        current_point = start
        
        for i in range(len(region_path) - 1):
            current_region = region_path[i]
            next_region = region_path[i + 1]
            
            connection = None
            for conn in self.region_connections[current_region]:
                if conn['region'] == next_region:
                    connection = conn
                    break
            
            if not connection:
                return None
                
            conn_point_current, conn_point_next = connection['connection_points'][0]
            
            path_in_region = self._local_astar(current_point, conn_point_current, current_region)
            if not path_in_region:
                return None
                
            full_path.extend(path_in_region[:-1])
            current_point = conn_point_next
            
        final_path = self._local_astar(current_point, goal, region_path[-1])
        if not final_path:
            return None
            
        full_path.extend(final_path)
        return full_path
    
    def _create_path_variation(self, path, original_paths):
        if not path or len(path) < 5:
            return None
            
        segment_start = np.random.randint(0, len(path) - 4)
        segment_end = np.random.randint(segment_start + 2, min(segment_start + 10, len(path) - 1))
        
        start_point = path[segment_start]
        end_point = path[segment_end]
        
        temp_avoid = self.points_to_avoid.copy()
        for i in range(segment_start + 1, segment_end):
            self.points_to_avoid.append(path[i])
            
        alt_path = self._full_astar(start_point, end_point)
        
        self.points_to_avoid = temp_avoid
        
        if alt_path and alt_path != path[segment_start:segment_end+1]:
            new_path = path[:segment_start] + alt_path + path[segment_end+1:]
            
            for orig in original_paths:
                if len(orig) == len(new_path) and all(a == b for a, b in zip(orig, new_path)):
                    return None
                    
            return new_path
            
        return None
    
    def _explore_quantum_paths(self, start, goal):
        paths = []
        
        if start == goal:
            return [[start]]
            
        start_region = self._get_region_for_point(start)
        goal_region = self._get_region_for_point(goal)
        
        if start_region is None or goal_region is None:
            return []
            
        if start_region == goal_region:
            path = self._local_astar(start, goal, start_region)
            if path:
                paths.append(path)
                
            while len(paths) < self.num_parallel_paths and paths:
                variation = self._create_path_variation(paths[0], paths)
                if variation:
                    paths.append(variation)
                else:
                    break
        else:
            region_path = self._region_level_search(start_region, goal_region)
            
            if not region_path:
                return []
                
            path = self._build_path_through_regions(region_path, start, goal)
            if path:
                paths.append(path)
                
            for i in range(len(region_path) - 1):
                if len(paths) >= self.num_parallel_paths:
                    break
                    
                current_region = region_path[i]
                next_region = region_path[i + 1]
                
                for conn in self.region_connections[current_region]:
                    if conn['region'] == next_region and len(conn['connection_points']) > 1:
                        for j in range(1, len(conn['connection_points'])):
                            if len(paths) >= self.num_parallel_paths:
                                break
                                
                            temp_conns = conn['connection_points'].copy()
                            conn['connection_points'] = [conn['connection_points'][j]] + temp_conns[:j] + temp_conns[j+1:]
                            
                            alt_path = self._build_path_through_regions(region_path, start, goal)
                            
                            conn['connection_points'] = temp_conns
                            
                            if alt_path and alt_path not in paths:
                                paths.append(alt_path)
            
            while len(paths) < self.num_parallel_paths and paths:
                variation = self._create_path_variation(paths[0], paths)
                if variation:
                    paths.append(variation)
                else:
                    break
                    
        return paths
    
    def _collapse_quantum_paths(self, paths):
        if not paths:
            return None
            
        best_score = float('inf')
        best_path = None
        
        for path in paths:
            score = len(path)
            
            for i, point in enumerate(path):
                if point in self.dynamic_obstacles:
                    obstacle_penalty = 100 * (1.0 - i/len(path))
                    score += obstacle_penalty
            
            if score < best_score:
                best_score = score
                best_path = path
                
        return best_path
    
    def _distance(self, p1, p2):
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
    
    def _generate_cache_key(self, start, goal):
        key_data = {
            'start': start,
            'goal': goal,
            'obstacles': sorted(self.dynamic_obstacles),
            'avoid': sorted(self.points_to_avoid)
        }
        json_str = json.dumps(key_data, sort_keys=True)
        return hashlib.md5(json_str.encode()).hexdigest()
    
    def find_path(self, start, goal):
        cache_key = self._generate_cache_key(start, goal)
        if cache_key in self.path_cache:
            return self.path_cache[cache_key]
            
        paths = self._explore_quantum_paths(start, goal)
        
        best_path = self._collapse_quantum_paths(paths)
        
        self.path_cache[cache_key] = best_path
        
        return best_path
    
    def update_dynamic_obstacles(self, obstacles):
        affected_regions = set()
        
        for obs in self.dynamic_obstacles:
            if obs not in obstacles:
                region = self._get_region_for_point(obs)
                if region is not None:
                    affected_regions.add(region)
        
        for obs in obstacles:
            if obs not in self.dynamic_obstacles:
                region = self._get_region_for_point(obs)
                if region is not None:
                    affected_regions.add(region)
        
        self.dynamic_obstacles = obstacles.copy()
        
        for region in affected_regions:
            self.invalidate_cache_region(region)
    
    def invalidate_cache_region(self, region_id):
        keys_to_remove = []
        
        for key, path in self.path_cache.items():
            if not path:
                continue
                
            for point in path:
                if self._get_region_for_point(point) == region_id:
                    keys_to_remove.append(key)
                    break
        
        for key in keys_to_remove:
            del self.path_cache[key]
    
    def set_points_to_avoid(self, points):
        self.points_to_avoid = points.copy()
        self.path_cache.clear()