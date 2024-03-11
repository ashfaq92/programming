import random
import math 
import numpy as np


def rand_float(min=0, max=1):
    """Generate a random float number within a given range."""
    return random.uniform(min, max)


def sigmoid(x):
    """Sigmoid function."""
    return 1 / (1 + math.exp(-x))


def initialize_weights(n_inputs, output_range):
    r = np.sqrt(1.0 / n_inputs)
    return np.random.uniform(-output_range / r, output_range / r, size=n_inputs)



