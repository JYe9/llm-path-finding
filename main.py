import openai
import os

# Set OpenAI API key from the environment
openai.api_key = os.getenv("OPENAI_API_KEY")

from llmastar.pather.a_star import AStar
from llmastar.pather.rrt import RRT
from llmastar.pather.llm_a_star import LLMAStar
from llmastar.pather.llm_rrt import LLMRRT

# Define the query
query = {"start": [5, 5], "goal": [10, 20], "size": [51, 31],
        "horizontal_barriers": [[10, 0, 25], [15, 30, 50]],
        "vertical_barriers": [[25, 10, 22]],
        "range_x": [0, 51], "range_y": [0, 31]}

# Tradictional Path Finding Algorithms
astar = AStar().searching(query=query, filepath='./output/astar.png')
rrt = RRT().searching(query=query, filepath='./output/rrt.png')

# Large Language Model Path Finding Algorithms
llm_a_star = LLMAStar(llm='gpt', prompt='standard').searching(query=query, filepath='./output/llm_astar.png')
llm_rrt = LLMRRT(llm='gpt', prompt='standard').searching(query=query, filepath='./output/llm_rrt.png')