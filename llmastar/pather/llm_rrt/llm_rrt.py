import json
import math
import random

from llmastar.env.search import env, plotting
from llmastar.model import ChatGPT, Llama3
from llmastar.utils import is_lines_collision, list_parse
from .prompt import *

class LLMRRT:
    """LLM-RRT algorithm with LLM suggestions to guide the tree exploration."""
    
    GPT_METHOD = "PARSE"
    GPT_LLMRRT_METHOD = "LLM-RRT"

    def __init__(self, llm='gpt', prompt='standard', goal_tolerance=5):
        self.llm = llm
        self.goal_tolerance = goal_tolerance  # Add a goal tolerance parameter
        if self.llm == 'gpt':
            self.parser = ChatGPT(method=self.GPT_METHOD, sysprompt=sysprompt_parse, example=example_parse)
            self.model = ChatGPT(method=self.GPT_LLMRRT_METHOD, sysprompt="", example=None)
        elif self.llm == 'llama':
            self.model = Llama3()
        else:
            raise ValueError("Invalid LLM model. Choose 'gpt' or 'llama'.")
        
        assert prompt in ['standard', 'cot', 'repe'], "Invalid prompt type. Choose 'standard', 'cot', or 'repe'."
        self.prompt = prompt

    def _parse_query(self, query):
        """Parse input query using the specified LLM model."""
        if isinstance(query, str):
            if self.llm == 'gpt':
                response = self.parser.chat(query)
                print(response)
                return json.loads(response)
            elif self.llm == 'llama':
                response = self.model.ask(parse_llama.format(query=query))
                print(response)
                return json.loads(response)
            else:
                raise ValueError("Invalid LLM model.")
        return query

    def _initialize_parameters(self, input_data):
        """Initialize environment parameters from input data."""
        self.s_start = tuple(input_data['start'])
        self.s_goal = tuple(input_data['goal'])
        self.horizontal_barriers = input_data['horizontal_barriers']
        self.vertical_barriers = input_data['vertical_barriers']
        self.range_x = input_data['range_x']
        self.range_y = input_data['range_y']
        self.Env = env.Env(self.range_x[1], self.range_y[1], self.horizontal_barriers, self.vertical_barriers)
        self.plot = plotting.Plotting(self.s_start, self.s_goal, self.Env)
        self.tree = {self.s_start: None}
        self.nodes = [self.s_start]
        self.step_size = 2 # RRT step size, could be parameterized

    def _initialize_llm_paths(self):
        """Initialize path suggestions using LLM guidance."""
        start, goal = list(self.s_start), list(self.s_goal)
        query = self._generate_llm_query(start, goal)

        if self.llm == 'gpt':
            response = self.model.ask(prompt=query, max_tokens=1000)
        elif self.llm == 'llama':
            response = self.model.ask(prompt=query)
        else:
            raise ValueError("Invalid LLM model.")

        nodes = list_parse(response)
        self.target_list = self._filter_valid_nodes(nodes)

        if not self.target_list or self.target_list[0] != self.s_start:
            self.target_list.insert(0, self.s_start)
        if not self.target_list or self.target_list[-1] != self.s_goal:
            self.target_list.append(self.s_goal)
        print(self.target_list)
    
    def _generate_llm_query(self, start, goal):
        """Generate the query for the LLM."""
        if self.llm == 'gpt':
            return gpt_prompt[self.prompt].format(start=start, goal=goal,
                                horizontal_barriers=self.horizontal_barriers,
                                vertical_barriers=self.vertical_barriers)
        elif self.llm == 'llama':
            return llama_prompt[self.prompt].format(start=start, goal=goal,
                                    horizontal_barriers=self.horizontal_barriers,
                                    vertical_barriers=self.vertical_barriers)

    def _filter_valid_nodes(self, nodes):
        """Filter out invalid nodes based on environment constraints."""
        return [(node[0], node[1]) for node in nodes
                if (node[0], node[1]) not in self.Env.obs
                and self.range_x[0] + 1 < node[0] < self.range_x[1] - 1
                and self.range_y[0] + 1 < node[1] < self.range_y[1] - 1]

    def searching(self, query, filepath='temp_rrt.png'):
        """RRT searching algorithm."""
        self.filepath = filepath
        print(query)
        input_data = self._parse_query(query)
        self._initialize_parameters(input_data)
        self._initialize_llm_paths()

        while True:
            s_random = self._get_random_node()
            s_nearest = self._get_nearest_node(s_random)
            s_new = self._steer(s_nearest, s_random)

            if not self.is_collision(s_nearest, s_new):
                self.tree[s_new] = s_nearest
                self.nodes.append(s_new)
                if self._reached_goal(s_new):
                    break

        path = self._extract_path()
        result = {
            "operation": len(self.tree),
            "length": sum(self._euclidean_distance(path[i], path[i+1]) for i in range(len(path)-1)),
            "llm_output": self.target_list
        }
        print(result)
        self.plot.animation(path, list(self.tree.keys()), True, "LLM-RRT", self.filepath)
        return result

    def _get_random_node(self):
        """Generate a random node within the search space."""
        return (random.randint(self.range_x[0], self.range_x[1]),
                random.randint(self.range_y[0], self.range_y[1]))

    def _get_nearest_node(self, s_random):
        """Find the nearest node in the tree to the random node."""
        return min(self.nodes, key=lambda s: self._euclidean_distance(s, s_random))

    def _steer(self, s_nearest, s_random):
        """Steer from the nearest node towards the random node."""
        theta = math.atan2(s_random[1] - s_nearest[1], s_random[0] - s_nearest[0])
        return (s_nearest[0] + self.step_size * math.cos(theta),
                s_nearest[1] + self.step_size * math.sin(theta))

    def _reached_goal(self, s_new):
        """Check if the new node is close enough to the goal using a tolerance."""
        if self._euclidean_distance(s_new, self.s_goal) < self.goal_tolerance:
            self.tree[self.s_goal] = s_new  # Explicitly add goal to the tree
            return True
        return False

    def _extract_path(self):
        """Extract the path by backtracking from goal to start."""
        path = [self.s_goal]
        node = self.s_goal
        print(f"Starting backtracking from goal: {self.s_goal}")
        
        while node != self.s_start:
            print(f"Current node: {node}")
            if node not in self.tree:
                print(f"KeyError: {node} not found in tree")
                print(f"Current tree keys: {list(self.tree.keys())}")
                raise KeyError(f"Node {node} not found in tree during backtracking")
            
            node = self.tree[node]
            print(f"Backtracked to node: {node}")
            path.append(node)
        
        print("Successfully extracted path")
        return path[::-1]

    def _euclidean_distance(self, p1, p2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def is_collision(self, s_start, s_end):
        """Check if the line segment (s_start, s_end) collides with any barriers."""
        line1 = [s_start, s_end]
        return any(is_lines_collision(line1, [[h[1], h[0]], [h[2], h[0]]]) for h in self.horizontal_barriers) or \
               any(is_lines_collision(line1, [[v[0], v[1]], [v[0], v[2]]]) for v in self.vertical_barriers) or \
               any(is_lines_collision(line1, [[x, self.range_y[0]], [x, self.range_y[1]]]) for x in self.range_x) or \
               any(is_lines_collision(line1, [[self.range_x[0], y], [self.range_x[1], y]]) for y in self.range_y)
