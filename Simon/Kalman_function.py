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
 
x0 = []
x1 = []
x2 = []
x_true = []
y_true = []
th_true = []
x = []
y = []
th = []

def JacobianA(x,u,dt):
    a1 = -u[0]*np.sin(x[2])*dt
    a2 = u[0]*np.cos(x[2])*dt

    JA = np.matrix([[ 1, 0, a1],
                    [ 0, 1,  a2],
                    [ 0, 0,    1]], dtype='float')
    # print('QQQ')
    # print(JA)
    # print('QQQ')
    return JA

def JacobianH(x,dt):
    # Should add noise to h here.
    JH = np.matrix([[x[0], 0, 0],
                    [0, x[1], 0],
                    [0, 0,   x[2]]], dtype='float')


    return JH

def prediction(X_hat_t_1,u,P_t_1,Q_t,dt):
    X_hat_t[0] = X_hat_t_1[0] + u[0]*np.cos(X_hat_t_1[2])*dt
    X_hat_t[1] = X_hat_t_1[1] + u[0]*np.sin(X_hat_t_1[2])*dt
    X_hat_t[2] = X_hat_t_1[2] + u[1]*dt  
    
    # Project the error covariance ahead
    P_t_1 = JacobianA(X_hat_t_1,u,dt)*P_t_1*JacobianA(X_hat_t_1,u,dt).T + Q_t
    # print(P_t_1)
    return X_hat_t, P_t_1
 

def update(X_hat_t,u,P_t,Z_t,R_t):
    hx = np.array([[float(X_hat_t[0])],
                    [float(X_hat_t[1])],
                    [float(X_hat_t[2])]])   

    S = np.matrix(JacobianH(X_hat_t,dt)*P_t*JacobianH(X_hat_t,dt).T + R_t, dtype='float')

    K = (P_t*JacobianH(X_hat_t,dt)) * np.linalg.inv(S)

    #print("K:\n",K)
    # Update the estimate via
    Z = Z_t.reshape(JacobianH(X_hat_t,dt).shape[0],1)
    #print("Z:\n",Z)
    y = Z - (hx)                         # Innovation or Residual
    X_t = X_hat_t + (K*y)
 
    # Update the error covariance
    I = np.eye(X_hat_t.shape[0])
    P_t = (I - (K*JacobianH(X_hat_t,dt)))*P_t
 
 
    x0.append(float(X_t[0]))
    x1.append(float(X_t[1]))
    x2.append(float(X_t[2]))
    return X_t,P_t,x0,x1,x2

############## MAIN CODE #######################

# Test scenatio
x_true = [0.0, 0.16, 0.32, 0.48, 0.65, 0.81, 0.97, 1.13, 1.29, 1.45, 1.61, 1.77, 1.94, 2.1, 2.26, 2.42, 2.58, 2.74, 2.9, 3.06, 3.23, 3.39, 3.55, 3.71, 3.87, 4.03, 4.19, 4.35, 4.52, 4.68, 4.84, 5.0, 5.16, 5.32, 5.48, 5.65, 5.81, 5.97, 6.13, 6.29, 6.45, 6.61, 6.77, 6.94, 7.1, 7.26, 7.42, 7.58, 7.74, 7.9, 8.06, 8.23, 8.39, 8.55, 8.71, 8.87, 9.03, 9.19, 9.35, 9.52, 9.68, 9.84, 10.0] #np.linspace(0, 9.99,33)
y_true = [11, 10.99, 10.96, 10.91, 10.84, 10.76, 10.65, 10.53, 10.39, 10.24, 10.08, 9.91, 9.72, 9.53, 9.34, 9.14, 8.94, 8.74, 8.55, 8.35, 8.17, 7.99, 7.82, 7.67, 7.53, 7.4, 7.29, 7.19, 7.12, 7.06, 7.02, 7.0, 7.0, 7.03, 7.07, 7.13, 7.21, 7.3, 7.42, 7.55, 7.69, 7.85, 8.02, 8.2, 8.39, 8.58, 8.78, 8.98, 9.17, 9.37, 9.57, 9.76, 9.94, 10.11, 10.27, 10.42, 10.55, 10.67, 10.77, 10.85, 10.92, 10.97, 10.99] # (np.cos(0.5*x_true))
#y_true = [y_true - 11 for x in y_true]
for i in range(len(x_true)):
    x.append(x_true[i] + 0.1*rnd.randint(-1,1))
    y.append(y_true[i] + 0.1*rnd.randint(-1,1))
    if x_true[i]<0.00001 and x_true[i]>-0.00001:
        th_true.append(np.arctan(y_true[i]/0.000001))
        th.append(th_true[i] + 0.1*rnd.randint(-2,2))        
    else:
        th_true.append(np.arctan(y_true[i]/x_true[i]))
        th.append(th_true[i] + 0.1*rnd.randint(-2,2))

th_true[0] = 0
th[0] = 0
print(th_true)

Y = np.vstack([x,y,th])

#Initial State cov
P_t = np.diag([1000.0, 1000.0, 1000.0])
R_t = np.diag([1000.0, 1000.0, 1000.0])
Q_t = np.diag([1000.0, 1000.0, 1000.0])

x_init = x_true[0]
y_init = y_true[0]
theta_init = th_true[0]
dt = 0.1

X_hat_t = np.matrix([[x_init, y_init, theta_init]]).T
measurements = Y #np.vstack((x[0],y[0],th[0]))

# Lenth of the measurement
m = measurements.shape[1]
uv_vec = [0.10008754734873981, 0.2024014730337683, 0.30504975634100856, 0.41012625063109537, 0.4453046244334794, 0.4453406970436493, 0.489436498612835, 0.4970069703300196, 0.5, 0.5, 0.5, 0.4949437429681709, 0.4314137827519757, 0.40210160790848254, 0.3704686515498797, 0.3424662406133058, 0.31915823809856086, 0.33641673904983765, 0.3566167172759019, 0.3930893842018862, 0.41839053709259283, 0.4612076276996193, 0.49531640719082914, 0.5, 0.5, 0.5, 0.5, 0.47983048551633134, 0.4257623203001033, 0.3887670438560679, 0.3653826349412731, 0.30952206957479744, 0.023658859574560397]
uw_vec = [-0.2015042745330413, -0.13671640993378195, -0.08767654571872797, -0.3199870939745476, 0.009117080351651761, -0.11607140945737993, -0.013215328305664086, -0.03453066267894815, 0.015240633035540182, 0.0007215920017030387, 0.10386936660190163, -0.005974970565937968, 0.1701437189728452, 0.07637616645080253, 0.32069841471138455, 0.07513688209166834, 0.4515321806865654, -0.022449155727095615, 0.3899550197539364, -0.012499354406197032, 0.1681368907454157, -0.030653412025078403, 0.1417011633890377, -0.0850583532613824, 0.05263232355836559, -0.07696983424745264, -0.007601769218124501, -0.11054070931576965, -0.08872014390918162, -0.25885599882479327, -0.015400240248662423, -0.5155888152385336, -1.0]

for i in range(33): #measurements.shape[1]):
#for i in range(3):
    uv = uv_vec[i]
    uw = uw_vec[i] 

    X_hat_t,P_hat_t = prediction(X_hat_t,np.vstack((uv,uw)),P_t,Q_t,dt)
    print("Prediction:")
    print("X_hat_t:\n",X_hat_t,"\nP_t:\n",P_hat_t)
   
    Z_t= np.vstack((x[i],y[i],th[i]))
    
    X_t,P_t,x0,x1,x2 = update(X_hat_t,np.vstack((uv,uw)),P_t,Z_t,R_t)
    print("Update:")
    print("X_t:\n",X_t,"\nP_t:\n",P_t)
    X_hat_t=X_t
    P_hat_t=P_t
    
    time.sleep(0.00005)


fig1 = plt.figure(figsize=(16,9))

# EKF State
#fig1=plt.quiver(x0,x1,np.cos(x2), np.sin(x2), color='#94C600', units='xy', width=100, scale=1)
fig1=plt.plot(x0,x1, label='EKF Position', c='k', lw=5)
print(x0)
# Measurements
# plt.scatter(mx[::5],my[::5], s=50, label='GPS Measurements', marker='+')

# Start/Goal
fig1=plt.scatter(x0[0],x1[0], s=60, label='Start', c='g')
fig1=plt.scatter(x0[-1],x1[-1], s=60, label='Goal', c='r')

fig1=plt.xlabel('X [m]')
fig1=plt.ylabel('Y [m]')
fig1=plt.title('Position')
fig1=plt.legend(loc='best')
fig1=plt.axis('equal')
fig1=plt.show()

fig2 = plt.figure(figsize=(16,9))
fig2=plt.plot(x_true,y_true, label='True Position', c='k', lw=5)
fig2=plt.xlabel('X [m]')
fig2=plt.ylabel('Y [m]')
fig2=plt.title('Position')
fig2=plt.legend(loc='best')
fig2=plt.axis('equal')
fig2=plt.show()

fig3 = plt.figure(figsize=(16,9))
fig3=plt.scatter(x,y, label='Noisy Position', c='k', lw=5)
fig3=plt.xlabel('X [m]')
fig3=plt.ylabel('Y [m]')
fig3=plt.title('Position')
fig3=plt.legend(loc='best')
fig3=plt.axis('equal')
fig3=plt.show()







