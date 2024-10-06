import openai
import os

# Set OpenAI API key from the environment
openai.api_key = os.getenv("OPENAI_API_KEY")

# from llmastar.pather import AStar, LLMAStar
from llmastar.pather.a_star import AStar
from llmastar.pather.llm_a_star import LLMAStar

query = {"start": [5, 5], "goal": [27, 15], "size": [51, 31],
        "horizontal_barriers": [[10, 0, 25], [15, 30, 50]],
        "vertical_barriers": [[25, 10, 22]],
        "range_x": [0, 51], "range_y": [0, 31]}
astar = AStar().searching(query=query, filepath='astar.png')
llm = LLMAStar(llm='gpt', prompt='standard').searching(query=query, filepath='llm.png')