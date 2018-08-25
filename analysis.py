#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 17 11:27:48 2018

@author: janko
"""

import numpy as np
import matplotlib.pyplot as plt

    
def get_values(filename):
    
    with open(filename, 'r') as f:
        lines = f.readlines()
        
    gt = []
    estimates = []
    rmse = []
    for l in lines:
        if 'GT' in l:
            l = l.strip('\n').split(' ')
            gt.append([float(temp) for temp in l[-4:]])
            
        if 'ES' in l:
            l = l.strip('\n').split(' ')
            estimates.append([float(temp) for temp in l[-4:]])
            
        if 'RMSE' in l:
            l = l.strip('\n').split(' ')
            rmse.append([float(temp) for temp in l[-4:]])
    
    gt = np.asarray(gt)
    estimates = np.asarray(estimates)
    rmse = np.asarray(rmse)
    
    return gt, estimates, rmse


_, _, rmse_all = get_values('build/output.txt')
_, _, rmse_radar = get_values('build/output_radar_only.txt')
_, _, rmse_laser = get_values('build/output_laser_only.txt')


plt.subplot(221)
plt.plot(rmse_all[:,0])
plt.plot(rmse_laser[:,0])
plt.plot(rmse_radar[:,0])
plt.legend(['Fusion', 'Laser', 'Radar'])
plt.title('x')
plt.grid(True)

plt.subplot(222)
plt.plot(rmse_all[:,1])
plt.plot(rmse_laser[:,1])
plt.plot(rmse_radar[:,1])
plt.legend(['Fusion', 'Laser', 'Radar'])
plt.title('y')
plt.grid(True)

plt.subplot(223)
plt.plot(rmse_all[:,2])
plt.plot(rmse_laser[:,2])
plt.plot(rmse_radar[:,2])
plt.legend(['Fusion', 'Laser', 'Radar'])
plt.title('vx')
plt.grid(True)

plt.subplot(224)
plt.plot(rmse_all[:,3])
plt.plot(rmse_laser[:,3])
plt.plot(rmse_radar[:,3])
plt.legend(['Fusion', 'Laser', 'Radar'])
plt.title('vy')
plt.grid(True)




gt_db2_init_laser, es_db2_init_laser, rmse_db2_init_laser = get_values('build/output_db2_init_laser.txt')
gt_db2_init_radar, es_db2_init_radar, rmse_db2_init_radar = get_values('build/output_db2_init_radar.txt')