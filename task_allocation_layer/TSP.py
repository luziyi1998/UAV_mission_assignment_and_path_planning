import numpy as np
from scipy.optimize import linprog
from scipy.optimize import minimize

cost_vector = np.array([1,2,3,-4,5])

def cost_mul(x, cost_vector=cost_vector, sign=1.0):
    sum = 0
    for i in range(len(cost_vector)):
        sum += x[i] * cost_vector[i]
    return -1 * sign * sum
cons = {

}
A_ub = np.array([[1,1,1,1,1]])
b_ub = np.array([3])
A_eq = np.array([[1, 1, 1, 1, 1]])
b_eq = np.array([3])
res = linprog(c=[1,1,1,1,1], A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=((0,1),(0,1),(0,1),(0,1),(0,1)),)
print(res)
