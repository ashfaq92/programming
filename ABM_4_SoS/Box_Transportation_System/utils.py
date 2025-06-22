import math
import random

""" GLOBAL CONSTANTS """
RAND_SEED = None
seeded_rand = random.Random(RAND_SEED)

"""Analysis parameters"""
DEBUG_MODE = False
DISTANCE_METHOD = "EUCLIDEAN"
ROBOT_PASSTHROUGH = True  # Change to True to let robots move through each other

"""CoCaRo simulation parameters"""
COLORS = ["RED", "GREEN", "BLUE"]
BOX_STATUSES = ["INITIALIZED", "CARRIED", "DEPOSITED"]
CARDINAL_DIRECTIONS = [(-1, 0), (0, -1), (1, 0), (0, 1)]  # Movement directions (cardinal only - no diagonals) left, up, right, down
INITIAL_ENERGY = 300
MAX_ENERGY = 300
ENERGY_COST = 1.0  
PERCEPTION_RADIUS = 3
BOX_GENERATION_RATE = 1  # every 3 time unit
SAME_COLOR_REWARD = int(2 * MAX_ENERGY / 3)
DIFFERENT_COLOR_REWARD = int(MAX_ENERGY / 3)
BASE_SPEED = 1.0

""" HELPER FUNCTIONS """
def validate_color(c):
    if c in COLORS:
        return c
    else:
        raise ValueError(f"Invalid color: {c}")

def calculate_distance(p1, p2):
    if DISTANCE_METHOD == "MANHATTAN":
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
    elif DISTANCE_METHOD == "EUCLIDEAN":
        return math.dist(p1, p2)
    else:
        raise ValueError('Invalid distance calculation method')


