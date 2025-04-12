import unittest
import numpy as np
from hqp.core import hqp

class TestHQP(unittest.TestCase):
    
    def setUp(self):
        maze = [
            [0, 1, 0, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 0, 1, 0],
            [1, 1, 0, 0, 0],
            [0, 0, 0, 1, 0]
        ]
        self.pathfinder = hqp(maze)

    def test_find_path(self):
        start = (0, 0)
        goal = (4, 4)
        path = self.pathfinder.find_path(start, goal)
        expected_path = [(0, 0), (0, 2), (1, 2), (2, 2), (3, 2), (4, 2), (4, 3), (4, 4)]
        self.assertEqual(path, expected_path)

    def test_no_path(self):
        start = (0, 1)
        goal = (4, 4)
        path = self.pathfinder.find_path(start, goal)
        self.assertIsNone(path)

    def test_dynamic_obstacles(self):
        start = (0, 0)
        goal = (4, 4)
        self.pathfinder.update_dynamic_obstacles([(2, 2)])
        path = self.pathfinder.find_path(start, goal)
        expected_path = [(0, 0), (0, 2), (1, 2), (1, 3), (2, 3), (3, 3), (4, 3), (4, 4)]
        self.assertEqual(path, expected_path)

    def test_points_to_avoid(self):
        start = (0, 0)
        goal = (4, 4)
        self.pathfinder.set_points_to_avoid([(1, 2), (3, 2)])
        path = self.pathfinder.find_path(start, goal)
        expected_path = [(0, 0), (0, 2), (1, 2), (2, 2), (3, 2), (4, 2), (4, 3), (4, 4)]
        self.assertEqual(path, expected_path)

if __name__ == '__main__':
    unittest.main()