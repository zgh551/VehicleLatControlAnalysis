# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 11:42:24 2020

@author: Henry Zhu
"""
import numpy as np
import control as ct
import matplotlib.pyplot as plt


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

# 产生曲线数据集
coefficient_a = 0.4
r = np.pi/coefficient_a
for i in np.arange(0.0,np.pi/coefficient_a,0.05):
    target_curvature_sets.x.append(i)
    target_curvature_sets.y.append(np.cos(coefficient_a*i) - 1)

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
    
plt.figure()
plt.title('Sliding Variable')
plt.xlabel('x[m]')
plt.plot(target_curvature_sets.x,target_curvature_sets.y)
plt.plot(target_curvature_sets.x,target_curvature_sets.yaw)
plt.plot(target_curvature_sets.x,target_curvature_sets.k)