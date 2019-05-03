import numpy as np
import scipy as sc
import scipy.optimize
from field_vision_functions import *
from numpy.random import randn
import matplotlib.pyplot as plt


# [field_vision] identifies location of crops vs path using
# (lidar) and publishes location relative to the center of the
# robot for all three crops to (crop_location)


# Set up the data to model
x = np.linspace(-10, 10)
y = three_peaks(x, -5 + randn(), randn(), 5 + randn())
crop_location_guess = [-5, 0, 5]  # we guess that our rows are at -5 and 5 relative to the heading based on prior info

# Align the data for modeling
xa, ya, sclr = align_data(tuple(x), tuple(y))
# Model the data
crop_guess_temp = find_rows(xa, ya, crop_location_guess)

# Plot aligned data and model
a, b, c = crop_guess_temp
plt.scatter(xa, ya)
plt.plot(x, three_peaks(x, a, b, c), '--r')
plt.scatter(crop_guess_temp, np.zeros(len(crop_guess_temp)))
plt.title("Alinged data")
plt.show()

# Convert data back
crop_guess_temp = convert_back(crop_guess_temp, sclr)
a, b, c = crop_guess_temp
print(crop_guess_temp)

# Plot the converted data
plt.scatter(x, y)
plt.plot(x, three_peaks(x, a, b, c), '--r')
plt.scatter(crop_guess_temp, np.zeros(len(crop_guess_temp)))
plt.title("Non-alinged data")
plt.show()
