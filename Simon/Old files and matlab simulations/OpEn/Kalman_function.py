import numpy as np
np.set_printoptions(threshold=3)
np.set_printoptions(suppress=True)
import random as rnd
from numpy import genfromtxt
import time 
import matplotlib.pyplot as plt
from scipy.stats import norm
# from sympy import Symbol, symbols, Matrix, sin, cos
# from sympy import init_printing
# from sympy.utilities.codegen import codegen
# init_printing(use_latex=True)
 
# x0 = []
# x1 = []
# x2 = []

def JacobianA(x,u,dt):
    a1 = -u[0]*np.sin(x[2])*dt
    a2 = u[0]*np.cos(x[2])*dt

    JA = np.matrix([[ 1, 0, a1],
                    [ 0, 1,  a2],
                    [ 0, 0,    1]], dtype='float')
    return JA

def JacobianH(x,dt):
    # Should add noise to h here.
    # JH = np.matrix([[x[0]*dt, 0, 0],
    #                 [0, x[1]*dt, 0],
    #                 [0, 0,   x[2]*dt]], dtype='float')
    JH = np.diag([1.0, 1.0, 1.0])

    return JH

def prediction(X_hat_t_1,u,P_t_1,Q_t,dt):
    X_hat_t  = np.matrix([[0,0,0]]).T 
    X_hat_t[0] = X_hat_t_1[0] + u[0]*np.cos(X_hat_t_1[2])*dt
    X_hat_t[1] = X_hat_t_1[1] + u[0]*np.sin(X_hat_t_1[2])*dt
    X_hat_t[2] = X_hat_t_1[2] + u[1]*dt  
    
    # Project the error covariance ahead
    P_t_1 = JacobianA(X_hat_t_1,u,dt)*P_t_1*JacobianA(X_hat_t_1,u,dt).T + Q_t
    # print(P_t_1)
    return X_hat_t, P_t_1
 

def update(X_hat_t,u,P_t,Z_t,R_t,dt):
    hx = np.array([[float(X_hat_t[0])],
                    [float(X_hat_t[1])],
                    [float(X_hat_t[2])]])   

    S = np.matrix(JacobianH(X_hat_t,dt)*P_t*JacobianH(X_hat_t,dt).T + R_t, dtype='float')

    K = (P_t*JacobianH(X_hat_t,dt)) * np.linalg.inv(S)

    Z = Z_t.reshape(JacobianH(X_hat_t,dt).shape[0],1)
    # print("Z:\n",Z)
    y = Z - (hx)                         # Innovation or Residual
    X_t = X_hat_t + (K*y)
 
    # Update the error covariance
    I = np.eye(X_hat_t.shape[0])
    P_t = (I - (K*JacobianH(X_hat_t,dt)))*P_t
    x0.append(float(X_t[0]))
    x1.append(float(X_t[1]))
    x2.append(float(X_t[2]))
    return X_t,P_t,x0,x1,x2

############## MAIN CODE ##########################################################################

############# HERE WE SHOULD GET THE INPUTS ###################
## MAKE SURE THIS IS IMPORTED:
# dt = 1
##

####### Test scenario, DELETE LATER!! :
# uv_vec = [0.10008754734873981, 0.2024014730337683, 0.30504975634100856, 0.41012625063109537, 0.4453046244334794, 0.4453406970436493, 0.489436498612835, 0.4970069703300196, 0.5, 0.5, 0.5, 0.4949437429681709, 0.4314137827519757, 0.40210160790848254, 0.3704686515498797, 0.3424662406133058, 0.31915823809856086, 0.33641673904983765, 0.3566167172759019, 0.3930893842018862, 0.41839053709259283, 0.4612076276996193, 0.49531640719082914, 0.5, 0.5, 0.5, 0.5, 0.47983048551633134, 0.4257623203001033, 0.3887670438560679, 0.3653826349412731, 0.30952206957479744, 0.023658859574560397];
# uw_vec = [-0.2015042745330413, -0.13671640993378195, -0.08767654571872797, -0.3199870939745476, 0.009117080351651761, -0.11607140945737993, -0.013215328305664086, -0.03453066267894815, 0.015240633035540182, 0.0007215920017030387, 0.10386936660190163, -0.005974970565937968, 0.1701437189728452, 0.07637616645080253, 0.32069841471138455, 0.07513688209166834, 0.4515321806865654, -0.022449155727095615, 0.3899550197539364, -0.012499354406197032, 0.1681368907454157, -0.030653412025078403, 0.1417011633890377, -0.0850583532613824, 0.05263232355836559, -0.07696983424745264, -0.007601769218124501, -0.11054070931576965, -0.08872014390918162, -0.25885599882479327, -0.015400240248662423, -0.5155888152385336, -1.0];

# Construct the hypothetic trajectory and add noise to the true path:
# x_true = [0]
# y_true = [11]
# th_true = [0]
# x = []
# y = []
# th = []

# for i in range(len(uv_vec)):
#     x_true.append( x_true[i]+uv_vec[i]*np.cos(th_true[i])*dt)
#     y_true.append(y_true[i]+uv_vec[i]*np.sin(th_true[i])*dt)
#     th_true.append(th_true[i]+uw_vec[i]*dt)
#     x.append(x_true[i] + 0.1*rnd.randint(-1,1))
#     y.append(y_true[i] + 0.1*rnd.randint(-1,1))
#     th.append(th_true[i] + 0.1*rnd.randint(-1,1))
################################################################

#Initial State cov
# P_t = np.diag([1000.0, 1000.0, 1000.0])
# R_t = np.diag([1000.0, 1000.0, 1000.0])
# Q_t = np.diag([1.0, 1.0, 1.0])

# x_init = x_true[0]
# y_init = y_true[0]
# theta_init = th_true[0]

# X_hat_t = np.matrix([[x_init, y_init, theta_init]]).T

# Get measurement:
# measurements = np.vstack([x,y,th])
# Length of the measurement
# m = measurements.shape[1]
m = []
x0 = []
x1 = []
x2 = [] 
def KalmanFilter(P_t,R_t,Q_t,X_hat_t,uv,uw,newMeasurement,dt):  
    #  newMeasurement = Vector with x y th
    # uv, uw are previously computed inputs
    # X_hat_t is the current position
    # P_t, Q_t, R_t are tuneable matrices
    x0 = []
    x1 = []
    x2 = []

    if len(m)<= 30:  # Save 15 measurment values
        m.append(np.vstack(newMeasurement))
    else:
        m.append(np.vstack(newMeasurement)) 
        m.pop(0)

    for i in range(len(m)):
        # uv = uv_vec[i]
        # uw = uw_vec[i] 
        
        X_hat_t,P_hat_t = prediction(X_hat_t,np.vstack((uv,uw)),P_t,Q_t,dt)

        Z_t= np.vstack((newMeasurement[0],newMeasurement[1],newMeasurement[2]))
        X_t,P_t,x0,x1,x2 = update(X_hat_t,np.vstack((uv,uw)),P_t,Z_t,R_t,dt)
        X_hat_t=X_t
        P_hat_t=P_t

    return x0[-1], x1[-1], x2[-1]

# fig1 = plt.figure(figsize=(16,9))

# EKF State
# fig1=plt.plot(x0,x1, label='EKF Position', c='k', lw=5)

# Start/Goal
# fig1=plt.scatter(x0[0],x1[0], s=60, label='Start', c='g')
# fig1=plt.scatter(x0[-1],x1[-1], s=60, label='Goal', c='r')

# fig1=plt.xlabel('X [m]')
# fig1=plt.ylabel('Y [m]')
# fig1=plt.title('Position')
# fig1=plt.legend(loc='best')
# fig1=plt.axis('equal')
# fig1=plt.show()

# fig2 = plt.figure(figsize=(16,9))
# fig2=plt.plot(x_true,y_true, label='True Position', c='k', lw=5)
# fig2=plt.xlabel('X [m]')
# fig2=plt.ylabel('Y [m]')
# fig2=plt.title('Position')
# fig2=plt.legend(loc='best')
# fig2=plt.axis('equal')
# fig2=plt.show()

# fig3 = plt.figure(figsize=(16,9))
# fig3=plt.scatter(x,y, label='Noisy Position', c='k', lw=5)
# fig3=plt.xlabel('X [m]')
# fig3=plt.ylabel('Y [m]')
# fig3=plt.title('Position')
# fig3=plt.legend(loc='best')
# fig3=plt.axis('equal')
# fig3=plt.show()
