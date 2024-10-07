import math
import random

from llmastar.env.search import env, plotting
from llmastar.utils import is_lines_collision

class RRT:
    """RRT builds a tree by randomly exploring the environment until it reaches the goal"""
    def __init__(self, step_size=2, max_iter=1000):
        self.step_size = step_size  # maximum step size for RRT
        self.max_iter = max_iter  # maximum iterations

    def searching(self, query, filepath='temp.png'):
        """
        RRT Searching.
        :return: path, visited order
        """
        self.filepath = filepath
        print(query)
        self.s_start = (query['start'][0], query['start'][1])
        self.s_goal = (query['goal'][0], query['goal'][1])

        self.horizontal_barriers = query['horizontal_barriers']
        self.vertical_barriers = query['vertical_barriers']
        self.range_x = query['range_x']
        self.range_y = query['range_y']
        self.Env = env.Env(self.range_x[1], self.range_y[1], self.horizontal_barriers, self.vertical_barriers)  # class Env
        self.plot = plotting.Plotting(self.s_start, self.s_goal, self.Env)
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.TREE = {self.s_start: None}  # tree structure to store nodes and their parents
        self.path = []
        self.iteration = 0

        # Start exploring
        for _ in range(self.max_iter):
            rand_node = self.random_sample()
            nearest_node = self.get_nearest_node(rand_node)
            new_node = self.steer(nearest_node, rand_node)

            if not self.is_collision(nearest_node, new_node):
                self.TREE[new_node] = nearest_node
                self.iteration += 1

                if self.distance(new_node, self.s_goal) < self.step_size:
                    self.TREE[self.s_goal] = new_node
                    break

        if self.s_goal in self.TREE:
            self.path = self.extract_path(self.TREE)
        else:
            print("No path found.")
        
        visited = list(self.TREE.keys())
        result = {"operation": self.iteration, "storage": len(self.TREE), "length": sum(self._euclidean_distance(self.path[i], self.path[i+1]) for i in range(len(self.path)-1))} 
        print(result)
        self.plot.animation(self.path, visited, True, "RRT", self.filepath)
        return result

    def random_sample(self):
        """Randomly samples a point within the defined environment."""
        return (random.uniform(self.range_x[0], self.range_x[1]),
                random.uniform(self.range_y[0], self.range_y[1]))

    def get_nearest_node(self, random_node):
        """Finds the nearest node in the tree to the randomly sampled node."""
        return min(self.TREE, key=lambda node: self.distance(node, random_node))

    def steer(self, from_node, to_node):
        """Steers from one node toward another, within a maximum step size."""
        distance = self.distance(from_node, to_node)
        if distance < self.step_size:
            return to_node
        
        theta = math.atan2(to_node[1] - from_node[1], to_node[0] - from_node[0])
        new_node = (from_node[0] + self.step_size * math.cos(theta),
                    from_node[1] + self.step_size * math.sin(theta))
        return new_node

    def distance(self, node1, node2):
        """Calculates Euclidean distance between two nodes."""
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

    def extract_path(self, TREE):
        """Extracts the path from the goal to the start node."""
        path = [self.s_goal]
        node = self.s_goal

        while node != self.s_start:
            node = TREE[node]
            path.append(node)

        return path[::-1]  # reverse the path

    def is_collision(self, s_start, s_end):
        """
        Check if the line segment (s_start, s_end) collides with any obstacles.
        :param s_start: start node
        :param s_end: end node
        :return: True if collision, False otherwise
        """
        line1 = [s_start, s_end]
        for horizontal in self.horizontal_barriers:
            line2 = [[horizontal[1], horizontal[0]], [horizontal[2], horizontal[0]]]
            if is_lines_collision(line1, line2):
                return True
        for vertical in self.vertical_barriers:
            line2 = [[vertical[0], vertical[1]], [vertical[0], vertical[2]]]
            if is_lines_collision(line1, line2):
                return True
        for x in self.range_x:
            line2 = [[x, self.range_y[0]], [x, self.range_y[1]]]
            if is_lines_collision(line1, line2):
                return True
        for y in self.range_y:
            line2 = [[self.range_x[0], y], [self.range_x[1], y]]
            if is_lines_collision(line1, line2):
                return True
        return False
    
    @staticmethod
    def _euclidean_distance(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p2[1] - p1[1]) ** 2)

