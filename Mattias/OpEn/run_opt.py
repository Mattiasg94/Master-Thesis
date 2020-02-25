from opt_build import ts, N, nu, nx,model_dd
import opengen as og
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation



mng = og.tcp.OptimizerTcpManager('python_test_build/model_dd_opt')
mng.start()
mng.ping()
# Init ego
(x_init, y_init, theta_init) = (0, 0, np.pi/4)
(xref, yref, thetaref) = (10, 10, 0)
plot_x = []
plot_y = []
plot_theta = []
plot_init_x=x_init
plot_init_y=y_init
# Init Obstacles
(x_obs, y_obs, theta_obs, r_obs) = (4.5, 4.5, 0, 1)
(v_obs, w_obs) = (0.1, 0.00001)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
def animate(i):
    ax.cla()
    ax.set_xlim((plot_init_x-0.5, xref+0.5))
    ax.set_ylim((plot_init_y-0.5, yref+0.5))
    ax.plot(X, Y, 'o', color='g')
    ax.plot(plot_x, plot_y, '-o', color='b')
    circle1 = plt.Circle((x_obs, y_obs), r_obs, color='r')
    ax.add_patch(circle1)


for i in range(N):
    plot_x.append(x_init)
    plot_y.append(y_init)
    plot_theta.append(theta_init)
    p_lst = [x_init, y_init, theta_init, xref, yref, thetaref]
    obs_1 = [x_obs, y_obs, theta_obs, v_obs, w_obs, r_obs]
    p_lst.extend(obs_1)
    solution = mng.call(p_lst, initial_guess=[1.0] * (nu*N))

    # Plot solution
    # ------------------------------------
    time = np.arange(0, ts*N, ts)
    u_star = solution['solution']
    print('solve_time_ms', solution['solve_time_ms'],
          'sek', solution['solve_time_ms']/1000)
    uv = u_star[0:nu*N:2]
    uw = u_star[1:nu*N:2]
    # Plot trajectory
    # ------------------------------------
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

    # ----------------- Plot -----------------
    ani = animation.FuncAnimation(fig, animate, interval=100000)
    plt.pause(0.001)
    # ----------------------------------------

    # Update init:
    (x_init, y_init, theta_init) = (X[1], Y[1], THETA[1])
    (x_obs, y_obs, theta_obs) = model_dd(x_obs, y_obs, theta_obs,v_obs,w_obs)

plt.show()
mng.kill()

