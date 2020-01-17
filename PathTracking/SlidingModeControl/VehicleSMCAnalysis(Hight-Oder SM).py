# -*- coding: utf-8 -*-
"""
Created on Thu Dec 19 20:20:31 2019

@author: zhuguohua
"""

import numpy as np
import control as ct
import matplotlib.pyplot as plt

c = 36

lambda_0 = 1.0
lambda_1 = 1.5
lambda_2 = 2.0

rho = 0.02
k = 1.1

eps = 0.05
Delta = 0.1
Alpha = 4

vref = 1
target_delta = 0.4
delta_rate = 4.0
dt = 0.02
global_delta = 0

total_x = 0

p1 = -1.478
p2 =  36.5
p3 = -358.6
p4 =  1752
p5 = -4257
p6 =  4128

#
# System state: x, y, psi , delta
# System input: v,delta_rate
# System output: x, y, psi, delta
# System parameters: wheelbase, maxsteer
#
def vehicle_update(t, x, u, params):
    # Get the parameters for the model
    l = params.get('wheelbase', 2.6)         # vehicle wheelbase
    delta_rate_max = params.get('maxsteer', 8)    # max steering angle (rad)
    # Saturate the steering input
    delta_rate = np.clip(u[1], -delta_rate_max, delta_rate_max)

    # Return the derivative of the state
    return np.array([
        np.cos(x[2]) * u[0],                # xdot = cos(psi) v
        np.sin(x[2]) * u[0],                # ydot = sin(psi) v
        (u[0] / l) * np.tan(x[3]),          # psi_dot = v/l tan(delta)
        delta_rate                          # delta_dot = u
    ])

def vehicle_output(t, x, u, params):
    return x                         # return x, y, psi (full state)

# Define the vehicle steering dynamics as an input/output system
vehicle = ct.NonlinearIOSystem(
    vehicle_update, vehicle_output, states=4, name='vehicle',
    inputs=('v', 'delta_rate'),
    outputs=('x', 'y', 'psi', 'delta'))

###############################################################################
# Control 
###############################################################################
def Sigmoid(x):
    return x/(np.fabs(x) + eps)

def Sat(x):
    val = 0;
    if x >= Delta:
        val = 1
    elif x <= -Delta:
        val = -1
    else:
        val = x/Delta
    return val
#
# System state: z0, z1, z2
# System input: ey, psi_r, psi
# System output: z0, z1, z2
# System parameters: none
#
def control_update(t, x, u, params):
    x1 = u[0]
    x2 = np.tan(u[1]) - np.tan(u[2])
    sigma = c*x1 + x2
    
    v0 = -14.7361*np.power(np.fabs(x[0] - sigma),2/3)*np.sign(x[0] - sigma) + x[1]
    v1 = -30*np.power(np.fabs(x[1] - v0),1/2)*np.sign(x[1] - v0) + x[2]
    v2 = -400*np.sign(x[2] - v1)
    return np.array([
        v0,         # z0_dot = v0
        v1,         # z1_dot = v1
        v2          # z2_dot = v2
    ])
#
# System state: z0, z1, z2
# System input: none
# System output: delta_rate
# System parameters: none
def control_output(t, x, u, params):    

    a = x[2] + 2*np.power(np.fabs(x[1]) + np.power(np.fabs(x[0]),2/3),-0.5)*(x[1] + np.power(np.fabs(x[0]),2/3)*np.sign(x[0]))
    b = np.fabs(x[2]) + 2*np.power(np.fabs(x[1]) + np.power(np.fabs(x[0]),2/3),0.5)
    delta_rate = Alpha*a/b
    
#    delta_rate = Alpha*np.sign(x[2] + 2*np.power(np.power(np.fabs(x[1]),3) + np.power(x[0],2),1/6)*np.sign(x[1] + np.power(np.fabs(x[0]),2/3)))
    
    return delta_rate

# Define the controller as an input/output system
controller = ct.NonlinearIOSystem(
    control_update, control_output, name='controller', states=3,    # static system
    inputs=('e_y' , 'psi_r', 'psi'),                                # system inputs
    outputs=('delta_rate')                                          # system outputs
)

###############################################################################
# Target
###############################################################################
# sin function
coefficient_a = 0.5
def TargetLine(x):
    return np.sin(coefficient_a*x) + 5

def TargetLineFirstDerivative(x):
    return coefficient_a*np.cos(coefficient_a*x)

def TargetLineSecondDerivative(x):
    return -coefficient_a*coefficient_a*np.sin(coefficient_a*x)


# circle
#def TargetLine(x):
#    return np.sqrt(25 - np.power(x,2))
#
#def TargetLineFirstDerivative(x):
#    return -x/np.sqrt(25 - np.power(x,2))
#
#def TargetLineSecondDerivative(x):
#    return -25.0*np.power((25 - np.power(x,2)),-1.5)

## zhl target line
    
#def TargetLine(x):
#    return p1*np.power(x,5) + p2*np.power(x,4) + p3*np.power(x,3) + p4*np.power(x,2) + p5*x + p6
#
#def TargetLineFirstDerivative(x):
#    return 5*p1*np.power(x,4) + 4*p2*np.power(x,3) + 3*p3*np.power(x,2) + 2*p4*x + p5
#
#def TargetLineSecondDerivative(x):
#    return (20*p1*np.power(x,3) + 12*p2*np.power(x,2) + 6*p3*x + 2*p4)


#
# System state: none
# System input: xref, vref
# System output: y_r, psi_r, delta_r ,v_r
# System parameters: none
#
def target_output(t, x, u, params):
    l = params.get('wheelbase', 2.6)
    
    y_r = TargetLine(u[0])
    psi_ref = np.arctan(TargetLineFirstDerivative(u[0]))
    delta_ref = np.arctan(l*np.power(np.cos(psi_ref),3)*TargetLineSecondDerivative(u[0]))
    return np.array([y_r,psi_ref,delta_ref,u[1]])

# Define the trajectory generator as an input/output system
target = ct.NonlinearIOSystem(
    None, target_output, name='target',
    inputs=('x_ref','v_ref'),
    outputs=('y_r', 'psi_ref', 'delta_ref' , 'v_r'))

###############################################################################
# System Connect
###############################################################################
LatSlidingModeControl = ct.InterconnectedSystem(
    # List of subsystems
    (target, controller, vehicle), name='LatSlidingModeControl',

    # Interconnections between  subsystems
    connections=(
        ('target.x_ref','vehicle.x'),  
        ('vehicle.v','target.v_r'),
        ('controller.e_y','target.y_r','-vehicle.y'),
        ('controller.psi_r','target.psi_ref'),
        ('controller.psi','vehicle.psi'),
        ('vehicle.delta_rate', 'controller.delta_rate'),
    ),

    # System inputs
    inplist=['target.v_ref'],
    inputs=['vref'],

    #  System outputs
    outlist=['vehicle.x', 'vehicle.y' , 'vehicle.psi','vehicle.delta'],
    outputs=['x', 'y', 'psi', 'delta']
)

###############################################################################
# Input Output Response
###############################################################################
# time of response
T = np.linspace(0,10,1000)
# the response
tout, yout = ct.input_output_response(LatSlidingModeControl, T, [vref*np.ones(len(T))],X0=[0.1,0.1,0.1,0.0,5.0,0.31,0.0])

#tout, yout = ct.input_output_response(vehicle, T, [vref*np.ones(len(T)),target_delta*np.ones(len(T)),delta_rate*np.ones(len(T))],X0=[0,5.0,0,0])

#plt.figure()
#plt.title('track')
#plt.xlabel('x[m]')
#plt.ylabel('y[m]')
#plt.plot(yout[0],yout[1])
#
#
#plt.figure()
#plt.title('delta')
#plt.xlabel('t[s]')
#plt.ylabel('delta[deg]')
#plt.plot(tout,yout[3])


target_y = []
target_psi = []
targte_curvature = []
for x in yout[0]:
    target_y.append(TargetLine(x))
    target_psi.append(np.arctan(TargetLineFirstDerivative(x)))
    targte_curvature.append(np.power(np.cos(np.arctan(TargetLineFirstDerivative(x))),3)*TargetLineSecondDerivative(x))
    
    
x1 = target_y - yout[1]

x2 = []
for i in range(len(tout)):
    x2.append(np.tan(yout[3][i]) - np.tan(target_psi[i]))

s = c*x1 + x2

plt.figure()
plt.title('Sliding Variable')
plt.xlabel('x[m]')
plt.plot(tout,s)

plt.figure()
plt.title('phase')
plt.xlabel('x1[m]')
plt.ylabel('x2[m]')
plt.plot(x1,x2)
plt.plot([-1,1],[c,-c])
 
plt.figure()
plt.title('Tracking')
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.plot(yout[0],yout[1])
plt.plot(yout[0],target_y)

plt.figure()
plt.title('pis angle')
plt.xlabel('x[m]')
plt.ylabel('psi[rad]')
plt.plot(yout[0],yout[2])

plt.figure()
plt.title('steering angle')
plt.xlabel('x[m]')
plt.ylabel('steering angle(deg/s)')
plt.plot(yout[0],yout[3]*16*57.3)

plt.figure()
plt.title('err of y')
plt.xlabel('x[m]')
plt.ylabel('ey[m]')
plt.plot(yout[0],target_y - yout[1])

plt.figure()
plt.title('err of psi')
plt.xlabel('x[m]')
plt.ylabel('e_psi[rad]')
plt.plot(yout[0],target_psi - yout[2])

plt.figure()
plt.xlabel('x[m]')
plt.title('curvature')
plt.plot(yout[0],targte_curvature)

delta_rate = []
for i in range(len(tout)):
    if i == 0:
        delta_rate.append(0)
    else:
        delta_rate.append(57.3*(yout[3][i] - yout[3][i-1])/(tout[i]-tout[i-1]))
        
plt.figure()
plt.xlabel('t[m]')
plt.title('delta_rate')
plt.plot(tout,delta_rate)

x = np.linspace(-1.0,1.0,1000)
sigmoid_y = []
for i in x:
    sigmoid_y.append(Sigmoid(i))
plt.figure()
plt.title("Sigmoid Function")
plt.xlabel("sigma")
plt.grid()
plt.plot(x,sigmoid_y)

sat_y = []
for i in x:
    sat_y.append(Sat(i))
plt.figure()
plt.title("Sat Function")
plt.xlabel("Sat")
plt.grid()
plt.plot(x,sat_y)



#plt.figure()
#plt.title("Sliding Variable")
#plt.xlabel("Time[s]")
#plt.plot(tout,c*yout[0]+yout[1])
#
#plt.figure() 
#plt.grid()
#plt.title("Asymptotic convergence for f(x,v,t)=sin(2t)")
#plt.xlabel("Time(s)")
#plt.plot(tout,yout[0],label='distance(m)')
#plt.plot(tout,yout[1],label='velocity(m/s)')
#plt.legend()
#plt.title('unit mass modle(without disturbance)')
#plt.figure()
#plt.title("Phase portrait")
#plt.xlabel("x")
#plt.ylabel("v")
#plt.plot(yout[0],yout[1])
#
#u = []
#for i in range(len(tout)):
#    u.append(c*yout[1][i] + rho*(c*yout[0][i] + yout[1][i])/(np.fabs(c*yout[0][i] + yout[1][i]) + eps))
#    
#plt.figure()
#plt.title("Sliding mode control")
#plt.xlabel("Time[s]")
#plt.plot(tout,u)
#plt.show()