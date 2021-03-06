# -*- coding: utf-8 -*-
"""
Revision 001:

- Kalman Filter function returns estimate xhat. 

Implement next: return np.mean(xhat) only if more than 150 readings 
 
"""
# Kalman filter function for FPGA implementation

import numpy as np

def Kalman_Filter(n_iter, Reading):

    # allocate space for arrays
    z = Reading
    xhat=np.zeros(n_iter)      # a posteri estimate of x
    P=np.zeros(n_iter)         # a posteri error estimate
    xhatminus=np.zeros(n_iter) # a priori estimate of x
    Pminus=np.zeros(n_iter)    # a priori error estimate
    K=np.zeros(n_iter)         # gain or blending factor
    
    # read hall sensor n_iter times
#    for i in range(0,n_iter):
#        z[i] = np.random.uniform(-0.6,0.6)
      #z[i] = add read_hall_sensor function here
    
    R = 0.1**2 # estimate of measurement variance, change to see effect
    Q = 1e-5 # process variance
    
    # intial guesses
    xhat[0] = z[0]
    P[0] = z[0]
    
    for k in range(1,n_iter):
        # time update
        xhatminus[k] = xhat[k-1]
        Pminus[k] = P[k-1]+Q
        
        # measurement update
        K[k] = Pminus[k]/( Pminus[k]+R )
        xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k])
        P[k] = (1-K[k])*Pminus[k]
    
    return round(xhat[-1],2)   # before return np.mean(z) 


#hall_sensor_value_estimate = Kalman_Filter(150)
#print('Estimated value: ', hall_sensor_value_estimate)