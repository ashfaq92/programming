import random

class Matrix:
    def __init__(self, rows=None, cols=None, es=None):
        self.rows = rows
        self.cols = cols
        self.es = es  # elements

def mat_at(m: Matrix, i: int, j: int):
    return m.es[(i) * m.cols + (j) ]


def mat_alloc(rows: int, cols: int):
    assert rows > 0
    assert cols > 0
    m = Matrix()
    m.rows = rows
    m.cols = cols
    m.es = [None] * rows * cols
    assert m.es is not None
    return m


def mat_dot(a: Matrix, b: Matrix):
    dst = mat_alloc(a.rows, b.cols)
    assert a.cols == b.rows
    n = a.cols  # inner value
    
    for i in range(dst.rows):
        for j in range(dst.cols):
            dst.es[i * dst.cols + j] = 0
            for k in range(n):
                dst.es[i * dst.cols + j] += mat_at(a, i, k) * mat_at(b, k, j)
    return dst


def mat_sum(dst: Matrix, a: Matrix):
    assert dst.rows == a.rows
    assert dst.cols == a.cols
    for i in range(dst.rows):
        for j in range(dst.cols):  # Iterate over the columns of the matrices
            dst.es[i * dst.cols + j] += a.es[i * a.cols + j]


def mat_print(name, m: Matrix):
    print(f"{name} = [")
    for i in range(m.rows):
        for j in range(m.cols):
            print(mat_at(m, i, j), end='\t')
        print()
    print("]")

def rand_val(low, high):
    return random.uniform(low or 0, high or 1)

def mat_rand(m: Matrix, low=None, high=None):
    for i in range(m.rows):
        for j in range(m.cols):
            m.es[i * m.cols + j] = rand_val(low, high)

def mat_fill(m: Matrix, val: float):
    for i in range(m.rows):
        for j in range(m.cols):
            m.es[i * m.cols + j] = val

def main():
    w1 = mat_alloc(2, 2)  # weight matrix
    b1 = mat_alloc(1, 2)  # bias matrix

    w2 = mat_alloc(2, 1)
    b2 = mat_alloc(1, 1)

    mat_rand(w1)
    mat_rand(b1)
    mat_rand(w2)
    mat_rand(b2)

    
    mat_print('w1', w1)
    print()
    mat_print('b1', b1)
    print()
    mat_print('w2', w2)
    print()
    mat_print('b2', b2)


if __name__ == "__main__":
    main()
