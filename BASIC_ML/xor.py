import repertoire as lib
from datasets import nor_gate_dataset as dataset
import random

class xor:
    def __init__(self):
        self.or_w1 = None
        self.or_w2 = None
        self.or_b = None

        self.nand_w1 = None
        self.nand_w2 = None
        self.nand_b = None

        self.and_w1 = None
        self.and_w2 = None
        self.and_b = None

    def assign_random_values(self):
        self.or_w1 = random.random()
        self.or_w2 = random.random()
        self.or_b = random.random()

        self.nand_w1 = random.random()
        self.nand_w2 = random.random()
        self.nand_b = random.random()

        self.and_w1 = random.random()
        self.and_w2 = random.random()
        self.and_b = random.random()

    def __str__(self):
        variables = (
            f"or_w1: {self.or_w1}\n"
            f"or_w2: {self.or_w2}\n"
            f"or_b: {self.or_b}\n"
            f"nand_w1: {self.nand_w1}\n"
            f"nand_w2: {self.nand_w2}\n"
            f"nand_b: {self.nand_b}\n"
            f"and_w1: {self.and_w1}\n"
            f"and_w2: {self.and_w2}\n"
            f"and_b: {self.and_b}"
        )
        return variables


# todo: might change random.random to rand_float

def forward(m, x1, x2):
    a = lib.sigmoid(m.or_w1 * x1 + m.or_w2 * x2 + m.or_b)
    b = lib.sigmoid(m.nand_w1 * x1 + m.nand_w2 * x2 + m.nand_b)
    c = lib.sigmoid(m.and_w1 * a + m.and_w2 * b + m.and_b)
    return c



def cost(m):
    result = 0
    for row in dataset:
        x1 = row[0]
        x2 = row[1]
        # y = lib.sigmoid(x1 * w1 + x2 * w2 + b)  
        y = forward(m, x1, x2)
        d = y - row[2]
        result += d*d
    return result / len(dataset)




def finite_diff(m, eps):
    g = xor()
    c = cost(m)
    
    # Store the original values
    original_or_w1 = m.or_w1
    original_or_w2 = m.or_w2
    original_or_b = m.or_b
    original_nand_w1 = m.nand_w1
    original_nand_w2 = m.nand_w2
    original_nand_b = m.nand_b
    original_and_w1 = m.and_w1
    original_and_w2 = m.and_w2
    original_and_b = m.and_b
    
    # Temporarily modify and calculate for or_w1
    m.or_w1 += eps
    g.or_w1 = (cost(m) - c) / eps
    m.or_w1 = original_or_w1  # Restore
    
    # Repeat for each weight and bias
    m.or_w2 += eps
    g.or_w2 = (cost(m) - c) / eps
    m.or_w2 = original_or_w2
    
    m.or_b += eps
    g.or_b = (cost(m) - c) / eps
    m.or_b = original_or_b
    
    m.nand_w1 += eps
    g.nand_w1 = (cost(m) - c) / eps
    m.nand_w1 = original_nand_w1
    
    m.nand_w2 += eps
    g.nand_w2 = (cost(m) - c) / eps
    m.nand_w2 = original_nand_w2
    
    m.nand_b += eps
    g.nand_b = (cost(m) - c) / eps
    m.nand_b = original_nand_b
    
    m.and_w1 += eps
    g.and_w1 = (cost(m) - c) / eps
    m.and_w1 = original_and_w1
    
    m.and_w2 += eps
    g.and_w2 = (cost(m) - c) / eps
    m.and_w2 = original_and_w2
    
    m.and_b += eps
    g.and_b = (cost(m) - c) / eps
    m.and_b = original_and_b
    
    # g now holds the gradient approximation for each parameter
    return g


def learn(m, g, rate):

    m.or_w1 -= g.or_w1 * rate
    m.or_w2 -= g.or_w2 * rate
    m.or_b -= g.or_b * rate
    m.nand_w1 -= g.nand_w1 * rate
    m.nand_w2 -= g.nand_w2 * rate
    m.nand_b -= g.nand_b * rate    
    m.and_w1 -= g.and_w1 * rate
    m.and_w2 -= g.and_w2 * rate
    m.and_b -= g.and_b * rate
    return m

def train(m, iter): 
    eps = 0.01
    rate = 0.1
    for _ in range(iter):
        g = finite_diff(m, eps)
        m = learn(m, g, rate)
        # print(cost(m))
    return m



def main(): 
    iter = 100000
    m = xor()
    m.assign_random_values()
    print(cost(m))

    m2 = train(m, iter)
    print(cost(m2))

    # for row in dataset:
    #     print(row[0], row[1], row[2], forward(m2, row[0], row[1]))

    # dissecting the neural net
    print("OR neuron")
    for x1 in range(2):
        for x2 in range(2):
            print(x1, x2, lib.sigmoid(m.or_w1 * x1 + m.or_w2 * x2 + m.or_b))

    print("NAND neuron")
    for x1 in range(2):
        for x2 in range(2):
            print(x1, x2, lib.sigmoid(m.nand_w1 * x1 + m.nand_w2 * x2 + m.nand_b))

    print("AND neuron")
    for x1 in range(2):
        for x2 in range(2):
            print(x1, x2, lib.sigmoid(m.and_w1 * x1 + m.and_w2 * x2 + m.and_b))




if __name__ == "__main__":
    main()
