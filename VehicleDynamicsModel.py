# -*- coding: utf-8 -*-
"""
Created on Thu Dec 19 20:20:31 2019

@author: zhuguohua
"""

import numpy as np
import control as ct
import matplotlib.pyplot as plt
#from scipy.misc import derivative


c = 25
rho = 0.9
k = 0.01
eps = 0.001

v_ref = 0.5
delta_ref = 0.5
delta_dot_ref = 8

# System state: x, y, psi
# System input: v, delta,delta_dot
# System output: x, y,psi
# System parameters: wheelbase, maxsteer
#
def vehicle_update(t, x, u, params):
    # Get the parameters for the model
    l             = params.get('wheelbase', 3.)         # vehicle wheelbase
    delta_max     = params.get('maxsteer', 0.5)         # max steering angle (rad)
    delta_dot_max = params.get('max_steer_rate', 8)     # max steering angle rate (rad/s)

    # Saturate the steering input
    delta     = np.clip(u[1], -delta_max, delta_max)
    delta_dot = np.clip(u[2], -delta_dot_max, delta_dot_max)

    if x[3] > delta:
        x[3] = delta
    elif x[3] < delta:
        x[3] = delta
    # Return the derivative of the state
    return np.array([
        np.cos(x[2]) * u[0],            # xdot = cos(psi) v
        np.sin(x[2]) * u[0],            # ydot = sin(psi) v
        (u[0] / l) * np.tan(x[3]),      # psi_dot = v/l tan(delta)
        delta_dot
    ])

def vehicle_output(t, x, u, params):
    return x                            # return x, y, psi (full state)

# Define the vehicle steering dynamics as an input/output system
vehicle = ct.NonlinearIOSystem(
    vehicle_update, vehicle_output, states=4, name='vehicle',
    inputs=('v', 'delta','delta_dot'),
    outputs=('x', 'y', 'psi','delta'))

###############################################################################
# Input Output Response
###############################################################################
# time of response
T = np.linspace(0, 40, 1000)

ref_Delta = delta_ref*np.ones(len(T))
ref_Delta[500:] = -delta_ref

ref_Delta_Dot = delta_dot_ref*np.ones(len(T))
ref_Delta_Dot[500:] = -delta_dot_ref
# the response
tout, yout = ct.input_output_response(vehicle, T, [v_ref*np.ones(len(T)),ref_Delta,ref_Delta_Dot],X0=[0,0,0,0])
    
plt.figure()
plt.plot(yout[0],yout[1])

plt.figure()
plt.plot(yout[0],yout[2])

plt.figure()
plt.plot(tout,yout[3])