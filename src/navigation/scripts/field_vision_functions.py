## for the important functions
import numpy as np
import scipy as sc
import scipy.optimize


def align_data(x,y):
    '''
    align_data()
        x : vector of x values, where 0 == robot center
        y : vector of distances from robot, where 0 == front of robot
    returns aligned data.
    '''
    n = len(x)
    phi = np.ones((n,2),dtype=float)
    phi[:,1] = x
    coefs = np.linalg.lstsq(phi,y)[0]
    y -= np.matmul(phi, coefs)
    min_y = min(y)
    max_y = max(y)
    y -= min_y
    y /= (max_y - min_y)

    min_x = min(x)
    max_x = max(x)
    x -= min_x
    x /= (max_x - min_x)
    
    scalers = np.array([[min_x, max_x],[min_y,max_y]])
    return 20*x - 10,y-1, scalers

def convert_back(x, scalers):
    x = (10+x)/20
    x *= scalers[0,1] - scalers[0,0]
    x += scalers[0,0]
    return x

def three_peaks(x,a,b,c):
    '''
    three_peaks()
        x : array, horizontal values
        a,b : centers of peaks
    evaluates a bi-modal gaussian with respect to 2 parameters,
    each specifying the center of one of the modes.
    '''
    return -np.exp(-(x-a)**2) - np.exp(-(x-b)**2) - np.exp(-(x-c)**2)

def jacobian(x,a,b,c):
    """
    Jacobian()
        x : array, horizontal values
        a,b : centers of three_peaks.
    returns the jacobian of the residual vector w/ respect to c.
    Useful for efficient optimization.
    """
    A = np.zeros((len(x),3))
    A[:,0] = -2*(x-a) * np.exp(-(x-a)**2)
    A[:,1] = -2*(x-b) * np.exp(-(x-b)**2)
    A[:,2] = -2*(x-c) * np.exp(-(x-c)**2)
    return A

def find_rows(x,y,centers):
    '''
    find_rows()
        x : array, independent values
        y : array, dependent values
        centers : length 3 array like, initial guess for row locations
    returns precise estimate of row location.
    '''
    return sc.optimize.curve_fit(three_peaks, x, y, p0=centers,method='lm')[0] # lm is used for efficiency!
