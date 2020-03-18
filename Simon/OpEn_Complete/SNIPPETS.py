from setup import *
from build_opt_traj import len_traj, model_dd
import opengen as og
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation
import math
import time
import serial
from Kalman_function import KalmanFilter

# ------ Init Bluetooth
# print("Start")
# port="COM13" #This will be different for various devices and on windows it will probably be a COM port.
# bluetooth=serial.Serial(port, 9600) #Start communications with the bluetooth unit
# print("Connected")
# bluetooth.flushInput() #This gives the bluetooth a little kick

def convert_input(uv,uw):
    # 2/0.07 =~ 28,57143
    v_r = 28.57143*uv + 28.57143*uw
    v_l = 28.57143*uv - 28.57143*uw
    return v_r,v_l


# Line 162
## Estimated position of EGO using model:
# X_hat_t = np.matrix([[x_init, y_init, theta_init]]).T 
# ## Estimated position of EGO using sensor: 
# newMeasurement = [0,0,0] # GET_MEASUREMENT()
# x_init, y_init, theta_init = KalmanFilter(P_t,R_t,Q_t,X_hat_t,u_star[0],u_star[1],newMeasurement,ts)    # Using u_star since uv uw isn't iniitalized prior to this line. Should us dt instead of ts later.

# bluetooth.write(b"<"+str.encode(str(v_r))+ b"," + str.encode(str(v_l)) + b">") #These need to be bytes not unicode, plus a number
# input_data=bluetooth.readline() #This reads the incoming data. In this particular example it will be the "Hello from Blue" line
# print(input_data.decode())  # Plot what the arduino board is getting. Incoming in bytes, thus decode
    

