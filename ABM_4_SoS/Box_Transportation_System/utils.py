import random

""" GLOBAL CONSTANTS """

RAND_SEED = None
seeded_rand = random.Random(RAND_SEED)

COLORS = ["RED", "GREEN", "BLUE"]
DEBUG_MODE = True
BOX_STATUSES = ["INITIALIZED", "CARRIED", "DEPOSITED"]

# CoCaRo simulation parameters
INITIAL_ENERGY = 300
MAX_ENERGY = 300
ENERGY_COST = 1.0  
PERCEPTION_RADIUS = 3
BOX_GENERATION_RATE = 1  # every 3 time unit
# reward/bonus options: 2/1, 15/10
REWARD_AMOUNT = 10.0
BONUS_AMOUNT = 5.0
BASE_SPEED = 1.5

""" HELPER FUNCTIONS """
def validate_color(c):
    if c in COLORS:
        return c
    else:
        raise ValueError(f"Invalid color: {c}")
    

def manhattan_distance(pos1, pos2):
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])