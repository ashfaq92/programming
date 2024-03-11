import math
import matplotlib.pyplot as plt

from datasets import train_data
from repertoire import rand_float
# Define train data where
# train_data[i][0] -> input to the model
# train_data[i][1] -> expected output





def cost(w, b=0):
    result = 0
    for row in train_data:
        x = row[0]
        # one neuron, one connection with bias=0 --> y = x * w + b
        y = x * w + b  # Model prediction
        d = y - row[1]  # Difference between predicted and expected
        result += d ** 2  # Sum of squared differences
    result /= len(train_data)
    return result

def train_rand(iter):
    costs = []
    optimal_result = math.inf
    optimal_param = None
    for _ in range(iter):
        w = rand_float(min=0, max=10)  # Random weight
        result = cost(w)
        # print('iter', i, 'Parameter:', w, 'error', result)
        if result < optimal_result:
            optimal_result = result
            optimal_param = w
            costs.append(result)

    return costs, optimal_result, optimal_param



def train_step(iter):
    costs = []
    eps = 1e-3
    w = rand_float(min=0, max=10)  
    optimal_result = math.inf
    for _ in range(iter):
        result = cost(w)
        if result < optimal_result:
            w -= eps
        else:
            w += eps
        optimal_result = result
        costs.append(result)

        # print(w, optimal_result)
    return costs, result, w

def train_step2(iter):
    lrate = 1e-3
    eps = 1e-3
    b = rand_float(min=0, max=10)  # bias
    w = rand_float(min=0, max=10)
    costs = []

    for _ in range(iter):
        costs.append(cost(w))
        c = cost(w, b)
        dw = ( cost(w + eps, b) - c ) / eps  # difference in weights
        db = ( cost(w, b + eps) - c ) / eps  # difference in bias
        w -= lrate * dw
        b -= lrate * db
        print('cost', c, 'w', w, 'b', b)
    
    return costs, cost(w), w



def main():
    iter = 10
    result_rand = train_rand(iter)
    result_step = train_step(iter)
    result_step2 = train_step2(iter)
    print(result_rand[1], result_rand[2])
    print(result_step[1], result_step[2])
    print(result_step2[1], result_step2[2])




    # Plotting the arrays
    plt.plot(result_rand[0], label='result_rand')
    plt.plot(result_step[0], label='result_step')
    plt.plot(result_step2[0], label='result_step2')

    # Adding a legend
    plt.legend()

    # Display the plot
    plt.show()


main()
