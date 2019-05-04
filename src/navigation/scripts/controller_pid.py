import numpy as np
import matplotlib.pyplot as plt
import math
import random


def static_vars(**kwargs):
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func

    return decorate


@static_vars(integral=0, previous_error=None)
def PID(error, dt, k_p, k_i, k_d, reset=False):
    """
    Proportional Integral Derivative controller
    u(t)= k_p*e(t) + k_i*integral(0-t, e(t)dt) + k_d*(d/dt)e(t)
    :param error: e(t)
    :param dt: time step between function runs
    :param k_p: proportional term
    :param k_i: integral term
    :param k_d: derivative term
    :param reset: if reset is ture the integral and previous error are
    back to 0 and None respectively
    :returns u(t):
    """
    # Set uninitialized error
    if PID.previous_error is None:
        PID.previous_error = error
    # Calculate integral
    PID.integral += error * dt
    # Calculate derivative
    derivative = (error - PID.previous_error) / dt
    # Calculate output
    output = k_p * error + k_i * PID.integral + k_d * derivative
    # Set previous_error
    PID.previous_error = error
    return output


if __name__ == "__main__":
    """
    Example 1: a ball with initial velocity and position try and have it match its y to y = ln(t)
    """
    # Create the ball
    ball_y = -5
    ball_vel_y = -1
    # Set up environment
    DT = .1  # run pid 10 times a second
    TIME = 20  # run for 20 seconds
    # Set up PID variables
    K_P = 1
    K_I = 0.5
    K_D = 1
    # Create numpy arrays to store data
    time_array = np.linspace(DT, TIME, int(TIME / DT))
    ln_array = np.log(time_array * DT)
    ball_array = []
    # Run PID
    # TODO: try PID before and after movement
    for i in range(0, int(TIME / DT)):
        # Plot ball
        ball_array.append(ball_y)
        # Move ball
        ball_y += ball_vel_y * DT
        # Run PID and modify ball_vel
        error = ln_array[i] - ball_y  # Calculate the error
        u = PID(error, DT, K_P, K_I, K_D)  # Find delta from the PID
        ball_vel_y += u  # Adjust velocity based on the PID
    plt.plot(time_array, ln_array)
    plt.plot(time_array, np.array(ball_array))
    plt.show()

    """
    Example 2: a ball with initial velocity and position try and have it match its y to y = sin(t), but this time 
    a random event will cause the balls velocity to change to a random value to see continuous correction
    """
    # Create the ball
    ball_y = 3
    ball_vel_y = 1
    # Set up environment
    DT = .1  # run pid 10 times a second
    TIME = 20  # run for 20 seconds
    # Set up PID variables
    K_P = 1
    K_I = 1
    K_D = 0.2
    # Create numpy arrays to store data
    time_array = np.linspace(DT, TIME, int(TIME / DT))
    ln_array = np.sin(2 * math.pi * time_array * DT)
    ball_array = []
    # Run PID
    # TODO: try PID before and after movement
    for i in range(0, int(TIME / DT)):
        if random.randrange(0, 3, 1) == 1:
            ball_vel_y = random.random()*-4 + 2
        # Plot ball
        ball_array.append(ball_y)
        # Move ball
        ball_y += ball_vel_y * DT
        # Run PID and modify ball_vel
        error = ln_array[i] - ball_y  # Calculate the error
        u = PID(error, DT, K_P, K_I, K_D)  # Find delta from the PID
        ball_vel_y += u  # Adjust velocity based on the PID
    plt.plot(time_array, ln_array)
    plt.plot(time_array, np.array(ball_array))
    plt.show()
