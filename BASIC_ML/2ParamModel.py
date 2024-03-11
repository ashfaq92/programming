from datasets import nand_gate_dataset as dataset
import datasets
import repertoire as lib
import matplotlib.pyplot as plt



def cost(w1, w2, b):
    result = 0
    for row in dataset:
        x1 = row[0]
        x2 = row[1]
        y = lib.sigmoid(x1 * w1 + x2 * w2 + b)  # Include bias in the calculation
        d = y - row[2]
        result += d*d
    return result / len(dataset)

def train(iter):
    lrate = 0.01
    eps = 1e-6
    w1 = lib.rand_float(-1, 1)
    w2 = lib.rand_float(-1, 1)
    b = lib.rand_float(-1, 1)  # Initialize bias
    costs = []  # List to store cost at each iteration
    for _ in range(iter):
        c = cost(w1, w2, b)
        costs.append(c)
        dw1 = ( cost(w1 + eps, w2, b) - c ) / eps
        dw2 = ( cost(w1, w2 + eps, b) - c ) / eps
        db = ( cost(w1, w2, b + eps) - c ) / eps  # Calculate gradient for bias
        w1 -= lrate * dw1
        w2 -= lrate * dw2
        b -= lrate * db  # Update bias
        # print('w1', w1, 'w2', w2, 'b', b, 'cost', c)
    
    return w1, w2, b, costs  # Return bias as part of the result

def main():
    w1, w2, b, costs = train(1000 * 1000)
    for row in datasets.xor_gate_dataset:
        print(row[0], row[1], lib.sigmoid(row[0] * w1 + row[1] * w2 + b), row[2])  # Adjusted to include bias in prediction
    plt.plot(costs)  # Plot the costs
    plt.xlabel('Iteration')
    plt.ylabel('Cost')
    plt.title('Cost Function Over Iterations')
    plt.show()
   

if __name__ == "__main__":
    main()