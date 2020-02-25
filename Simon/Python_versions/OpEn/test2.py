from opt_build import ts, N, nu, nx
import opengen as og
import numpy as np
#----------------- Plot -----------------
from matplotlib.pylab import *
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation
#---------------------------------------- 

mng = og.tcp.OptimizerTcpManager('python_test_build/model_dd_opt')
mng.start()

mng.ping()

# Initial values
(x_init, y_init, theta_init) = (-1, -1, np.pi/4)
plot_x = []
plot_y = []
plot_theta = []

#----------------- Plot -----------------
#style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
def animate(i):
    ax1.cla()
    ax1.set_xlim(-1,5)
    ax1.set_ylim(-1,5)
    ax1.plot(X,Y,'o', color='g')
    ax1.plot(plot_x,plot_y,'-o', color='b')
    circle1 = plt.Circle((0, 0), 0.5, color='r')
    ax1.add_patch(circle1)

#----------------------------------------    

for k in range(0,60,1):
    plot_x.append(x_init)
    plot_y.append(y_init)
    plot_theta.append(theta_init)

    # print(plot_x)
    solution = mng.call([x_init, y_init, theta_init], initial_guess=[1.0] * (nu*N))
    time = np.arange(0, ts*N, ts)
    u_star = solution['solution']
    ux = u_star[0:nu*N:2]
    uy = u_star[1:nu*N:2]

    X = [0.0]*(N+1)
    X[0] = x_init
    Y = [0.0]*(N+1)
    Y[0] = y_init
    THETA = [0.0]*(N+1)
    THETA[0] = theta_init

    for t in range(0, N):
        u_t = u_star[t*nu:(t+1)*nu]
        x = X[t]
        y = Y[t]
        theta = THETA[t]
        X[t+1] = x + ts*np.cos(theta)*u_t[0]
        Y[t+1] = y + ts*np.sin(theta)*u_t[0]
        THETA[t+1] = theta + ts*u_t[1]

    print(k)

    #----------------- Plot -----------------
    ani = animation.FuncAnimation(fig,animate,interval=100000)
    plt.pause(0.001)
    #----------------------------------------   

    # Update init:
    (x_init, y_init, theta_init) = (X[1], Y[1],THETA[1])
  
plt.show()
mng.kill()
