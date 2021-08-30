# -*- coding: utf-8 -*-
"""
Created on Thu Dec 19 20:20:31 2019

@author: Henry Zhu
"""

import numpy as np
import control as ct
import matplotlib.pyplot as plt

simulation_time = 40

# steering control parameter
K_e = 0.5

# reference velocity and the target steering delta angle 
v_ref = 0.4

# vehicle init position
init_x = 0.0 - 2.6
init_y = 0.0
init_yaw = 0.0

# angle to -pi-pi
def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
###############################################################################
# Target
###############################################################################
# 定义目标曲线结构
class TargetCurvature:
    def __init__(self,x=0.0,y=0.0,yaw=0.0,k=0.0,v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.k   = k
        self.v   = v
  
# 声明目标曲线数组      
target_curvature_sets = TargetCurvature(x=[], y= [], yaw=[], k = [], v=[])

# cos function
coefficient_a = 0.4
def COS_Target_Line(index):
    return index,np.cos(coefficient_a*index) - 1

radius = 6
def Circle_Target_Line(index):
    return radius*np.cos(index + np.pi*0.5),(radius*np.sin(index + np.pi*0.5) - radius)

# 产生曲线数据集
#for i in np.arange(0,simulation_time*v_ref+3,0.05):
#    x,y = COS_Target_Line(i)
#    target_curvature_sets.x.append(x)
#    target_curvature_sets.y.append(y)
#    target_curvature_sets.v.append(v_ref)
    
for i in np.arange(0,-(simulation_time*v_ref + 3)/radius,-0.05):
    x,y = Circle_Target_Line(i)
    target_curvature_sets.x.append(x)
    target_curvature_sets.y.append(-y)
    target_curvature_sets.v.append(v_ref)

# 求取目标曲线的斜率
for i in range(len(target_curvature_sets.x)):
    if i == 0:
        dx = target_curvature_sets.x[i+1] - target_curvature_sets.x[i]
        dy = target_curvature_sets.y[i+1] - target_curvature_sets.y[i]
        ddx = target_curvature_sets.x[2] + target_curvature_sets.x[0] - 2*target_curvature_sets.x[1]
        ddy = target_curvature_sets.y[2] + target_curvature_sets.y[0] - 2*target_curvature_sets.y[1]
    elif i == (len(target_curvature_sets.x)-1):
        dx = target_curvature_sets.x[i] - target_curvature_sets.x[i-1]
        dy = target_curvature_sets.y[i] - target_curvature_sets.y[i-1]
        ddx = target_curvature_sets.x[i] + target_curvature_sets.x[i-2] - 2*target_curvature_sets.x[i-1]
        ddy = target_curvature_sets.y[i] + target_curvature_sets.y[i-2] - 2*target_curvature_sets.y[i-1]
    else:      
        dx = target_curvature_sets.x[i+1] - target_curvature_sets.x[i]
        dy = target_curvature_sets.y[i+1] - target_curvature_sets.y[i]
        ddx = target_curvature_sets.x[i+1] + target_curvature_sets.x[i-1] - 2*target_curvature_sets.x[i]
        ddy = target_curvature_sets.y[i+1] + target_curvature_sets.y[i-1] - 2*target_curvature_sets.y[i]
        
    target_curvature_sets.yaw.append(np.arctan2(dy,dx))
    target_curvature_sets.k.append((ddy*dx-ddx*dy)/(np.power(np.power(dx,2)+np.power(dy,2),1.5)))
   

#
# System state: none
# System input: v_ref, x, y
# System output: x_r, y_r, yaw_r, k_r, v_r
# System parameters: none
#
def target_output(t, x, u, params):
    ex = [u[1] - icx for icx in target_curvature_sets.x]
    ey = [u[2] - icy for icy in target_curvature_sets.y]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(ex, ey)]

    mind = min(d)

    index = d.index(mind)
    
    return np.array([target_curvature_sets.x[index],target_curvature_sets.y[index],target_curvature_sets.yaw[index],target_curvature_sets.k[index],u[0]])

# Define the trajectory generator as an input/output system
target = ct.NonlinearIOSystem(
    None, target_output, name='target',
    inputs=('v_ref', 'x', 'y'),
    outputs=('x_r', 'y_r', 'yaw_r', 'k_r' , 'v_r'))

###############################################################################
# Control 
###############################################################################
#
# System state: none
# System input:  e_x, e_y, e_yaw, yaw_ref, k_ref, v_ref
# System output: delta, v_r
# System parameters: l
def control_output(t, x, u, params):
    err_crs = np.cos(u[3])*u[1] - np.sin(u[3])*u[0]
    
    err_yaw = pi_2_pi(u[2])
    
    if u[5] == 0:
        return  np.array([0.0, u[5]])
    
    delta_ctl = err_yaw + np.arctan(K_e*err_crs/u[5])
    
    return  np.array([delta_ctl, u[5]])

# Define the controller as an input/output system
controller = ct.NonlinearIOSystem(
    None, control_output, name='controller',                        # static system
    inputs=('e_x', 'e_y' ,'e_yaw' ,'yaw_ref', 'k_ref', 'v_ref'),    # system inputs
    outputs=('delta', 'v')                                          # system outputs
)

###############################################################################
# Kenamic of Vehicle Plant
###############################################################################
# System state: x, y, yaw
# System input: delta, v
# System output: x, y, yaw
# System parameters: wheelbase, maxsteer
# 
def vehicle_update(t, x, u, params):
    # Get the parameters for the model
    l = params.get('wheelbase', 2.6)         # vehicle wheelbase
    delta_max = params.get('maxsteer', 0.5)    # max steering angle (rad)
    # Saturate the steering input
    delta = np.clip(u[0], -delta_max, delta_max)

    # Return the derivative of the state
    return np.array([
        np.cos(x[2]) * u[1],                # xdot = cos(psi) v
        np.sin(x[2]) * u[1],                # ydot = sin(psi) v
        (u[1] / l) * np.tan(delta)          # delta_dot = v/l tan(delta)   
    ])

def vehicle_output(t, x, u, params):
    l = params.get('wheelbase', 2.6) 
    front_wheel_x = x[0] + l*np.cos(x[2])
    front_wheel_y = x[1] + l*np.sin(x[2])
    return np.array([front_wheel_x,front_wheel_y,pi_2_pi(x[2])])                                # return x, y, psi (full state)

# Define the vehicle steering dynamics as an input/output system
vehicle = ct.NonlinearIOSystem(
    vehicle_update, vehicle_output, states=3, name='vehicle',
    inputs=('delta', 'v'),
    outputs=('x', 'y', 'yaw'))

###############################################################################
# System Connect
###############################################################################
LatRearWheelFeedbackControl = ct.InterconnectedSystem(
    # List of subsystems
    (target, controller, vehicle), name='LatRearWheelFeedbackControl',

    # Interconnections between  subsystems
    connections=(
        ('target.x','vehicle.x'),
        ('target.y','vehicle.y'),
        ('controller.e_x','target.x_r','-vehicle.x'), # e_x:x轴方向偏差
        ('controller.e_y','target.y_r','-vehicle.y'), # e_y:y轴方向偏差
        ('controller.e_yaw','target.yaw_r','-vehicle.yaw'), # e_yaw:偏航角偏差
        ('controller.yaw_ref','target.yaw_r'),
        ('controller.k_ref','target.k_r'),
        ('controller.v_ref','target.v_r'),
        ('vehicle.delta', 'controller.delta'),
        ('vehicle.v', 'controller.v')
    ),

    # System inputs
    inplist=['target.v_ref'],
    inputs=['v_ref'],

    #  System outputs
    outlist=['vehicle.x', 'vehicle.y' , 'vehicle.yaw','controller.delta'],
    outputs=['x', 'y', 'yaw', 'delta']
)

###############################################################################
# Input Output Response
###############################################################################
# time of response
T = np.linspace(0,simulation_time,2000)
# the response
tout, yout = ct.input_output_response(LatRearWheelFeedbackControl, T, [v_ref*np.ones(len(T))],X0=[init_x,init_y,init_yaw])

 
err_curvature = []
for i in range(len(tout)):
    ex = [yout[0][i] - icx for icx in target_curvature_sets.x]
    ey = [yout[1][i] - icy for icy in target_curvature_sets.y]
    
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(ex, ey)]
    mind = min(d)        
    index = d.index(mind)
    
    err_curvature.append(np.sqrt(mind))
 
plt.figure()
plt.title('Tracking')

plt.subplot(2,1,1)
plt.grid()
plt.title('Position Track')
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.plot(yout[0],yout[1],label="track_path")
plt.plot(target_curvature_sets.x,target_curvature_sets.y,label="target_curvature")
plt.legend()
plt.subplot(2,1,2)
plt.grid()
plt.title('Yaw Track')
plt.xlabel('x[m]')
plt.ylabel('err[m]')
plt.plot(err_curvature,label="err")
plt.legend()

plt.figure()
plt.grid()
plt.title('Yaw angle')
plt.xlabel('x[m]')
plt.ylabel('psi[rad]')
plt.plot(tout,yout[2])
#plt.plot(tout,yout[4])

plt.figure()
plt.grid()
plt.title('steering angle')
plt.xlabel('x[m]')
plt.ylabel('steering angle(deg/s)')
plt.plot(tout,yout[3]*16*57.3)

plt.figure()
plt.title('curvature')
plt.xlabel('x[m]')
plt.plot(target_curvature_sets.y)
plt.plot(target_curvature_sets.yaw)
plt.plot(target_curvature_sets.k)