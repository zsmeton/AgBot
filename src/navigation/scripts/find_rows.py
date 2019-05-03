import numpy as np
from numpy.random import randn
import scipy as sc
import scipy.optimize
import matplotlib.pyplot as plt


def place_peaks(x, a, b, r=1):
    """
    place_peaks(x,a,b,r=1)
        x : array like
        a : float
        b : float
        r : float
    constructs a bi-modal negative absolute value at locations
    x = a, and x = b, with radius r: ____/\\_______/\\__ sort of shape
                                          a        b
    """
    n = len(x)
    y = np.zeros(n)
    for i in range(n):
        if (a - r <= x[i] and x[i] <= a + r):
            y[i] = r - abs(x[i] - a)
        elif (b - r <= x[i] and x[i] <= b + r):
            y[i] = r - abs(x[i] - b)
    return -y + 0.01 * randn(n)


def bi_modal_gaussian(c):
    '''
    g(c)
        c : length 2 array like
    evaluates a bi-modal gaussian with respect to 2 parameters,
    each specifying the center o/home/zsmeton/Downloads/example.pyf one of the modes.
    '''
    return -np.exp(-(x - c[0]) ** 2) - np.exp(-(x - c[1]) ** 2)


def residual_vector(c):
    """
    f(c)
        c : length 2 array like
    returns the residual vector g(c) - y.
    Is the objective function for minimization.
    """
    return bi_modal_gaussian(c) - y


def jacobian(c):
    """
    J(c)
        c : length 2 array like
    returns the jacobian of the residual vector w/ respect to c.
    Useful for efficient optimization.
    """
    A = np.zeros((len(x), 2))
    A[:, 0] = -2 * (x - c[0]) * np.exp(-(x - c[0]) ** 2)
    A[:, 1] = -2 * (x - c[1]) * np.exp(-(x - c[1]) ** 2)
    return A


def find_rows(c):
    '''
    find_rows(c)
        c : length 2 array like, initial guess for row locations
    returns precise estimate of row location.
    '''
    return sc.optimize.least_squares(residual_vector, c, jac=jacobian, method='lm').x  # lm is used for efficiency!


x = np.linspace(-10, 10)
y = place_peaks(x, -5 + randn(), 5 + randn(), 1)

### Example of how it might be used,
c = [-5, 5]  # we guess that our rows are at -5 and 5 relative to the heading based on prior info
c = find_rows(c)  # we find the actual position


### here we plot to show off
plt.plot(x, y, 'o', markersize=3)
plt.plot(x, bi_modal_gaussian(c), '--r')
plt.show()
print("predicted centers : ", round(c[0], 3), round(c[1], 3))
