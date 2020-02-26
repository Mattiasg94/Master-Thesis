from opt_build import ts, N, nu, nx, model_dd
import opengen as og
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation


mng = og.tcp.OptimizerTcpManager('python_test_build/model_dd_opt')
mng.start()
mng.ping()
# -------Init ego
(x_init, y_init, theta_init) = (0, 0, np.pi/4)
(xref, yref, thetaref) = (10, 10, 0)
# -------Init Obstacles
(x_obs, y_obs, theta_obs, r_obs) = (2, 5, 0, 1.9)
(v_obs, w_obs) = (0.3, 0.00001)
# -------Init plot
plot_x = []
plot_y = []
plot_theta = []
plot_init_x = x_init
plot_init_y = y_init
total_sec = 0
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

u_star=[0.0] * (nu*N)
for i in range(60):
    plot_x.append(x_init)
    plot_y.append(y_init)
    plot_theta.append(theta_init)
    p_lst = [x_init, y_init, theta_init, xref, yref, thetaref]
    obs_1 = [x_obs, y_obs, theta_obs, v_obs, w_obs, r_obs]
    p_lst.extend(obs_1)
    u_star=[0.0] * (nu*N)
    solution = mng.call(p_lst, initial_guess=u_star)

    u_star = solution['solution']
    if not solution['exit_status'] == 'Converged':
        print('---------------------')
        print('exit_status', solution['exit_status'])
        # print('num_outer_iterations', solution['num_outer_iterations'])
        print('f1_infeasibility', solution['f1_infeasibility'])
        # print('last_problem_norm_fpr', solution['last_problem_norm_fpr'])
        # print('penalty', solution['penalty'])

    print('solve_time_ms', round(solution['solve_time_ms'], 2), 'sek', round(
        solution['solve_time_ms']/1000, 2))
    total_sec += solution['solve_time_ms']/1000

    # -------Plot trajectory
    uv = u_star[0:nu*N:2]
    uw = u_star[1:nu*N:2]
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

    ani = animation.FuncAnimation(fig, animate, interval=100000)
    

    # Update init:
    (x_init, y_init, theta_init) = (X[1], Y[1], THETA[1])
    (x_obs, y_obs, theta_obs) = model_dd(x_obs, y_obs, theta_obs, v_obs, w_obs)
    plt.pause(0.0001)
print('total_sec', total_sec)
plt.show()
mng.kill()
