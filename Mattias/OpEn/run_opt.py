from opt_build import ts, N, nu, nx, model_dd
import opengen as og
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation
import math
mng = og.tcp.OptimizerTcpManager('python_test_build/model_dd_opt')
mng.start()
mng.ping()
# -------Init ego
(x_init, y_init, theta_init) = (0, 0, np.pi/4)
(xref, yref, thetaref) = (10, 10, 0)
(v_init, w_init) = (0, 0)
# -------Init Obstacles
(x_obs, y_obs, theta_obs, r_obs) = (3, 5, 0, 1)
(v_obs, w_obs) = (0.0, 0.00001)
# -------Init plot
plot_x = []
plot_y = []
plot_theta = []
plot_init_x = x_init
plot_init_y = y_init
total_sec = 0
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)


def dist_to_ref(x_init, y_init):
    ref_abs_lst = []
    for i in range(len(X_REF)):
        ref_abs_lst.append(math.sqrt((X_REF[i]-x_init)**2+(Y_REF[i]-y_init)**2))
    min_val=min(ref_abs_lst)
    idx=ref_abs_lst.index(min_val)
    return idx

def animate(i):
    ax.cla()
    ax.set_xlim((plot_init_x-0.5, xref+0.5))
    ax.set_ylim((plot_init_y-0.5, yref+0.5))
    ax.plot(X, Y, 'o', color = 'g')
    ax.plot(plot_x, plot_y, '-o', color = 'b')
    ax.plot(xtraj, ytraj, '--', color = 'y',)
    #ax.plot(X_REF, Y_REF, '--', color = 'y',)
    circle1=plt.Circle((x_obs, y_obs), r_obs, color = 'r')
    ax.add_patch(circle1)

Y_REF=[0, 1.9, 4.2, 6.1, 7.7, 8.9, 9.6, 10, 10, 10, 10]
X_REF=[0, 1.0, 2.1, 3.6, 5.5, 7.5, 8.9, 10, 10, 10, 10]
u_star=[0.0] * (nu*N)
for i in range(40):
    plot_x.append(x_init)
    plot_y.append(y_init)
    plot_theta.append(theta_init)
    p_lst=[x_init, y_init, theta_init, v_init, w_init, xref, yref, thetaref]
    obs_1=[x_obs, y_obs, theta_obs, v_obs, w_obs, r_obs]
    p_lst.extend(obs_1)
    idx= dist_to_ref(x_init, y_init)
    xtraj=X_REF[idx+1:idx+4]
    ytraj=Y_REF[idx+1:idx+4]
    p_lst.extend(xtraj)
    p_lst.extend(ytraj)

    u_star=[0.0] * (nu*N)

    solution=mng.call(p_lst, initial_guess=u_star)
    (x_obs, y_obs, theta_obs)=model_dd(x_obs, y_obs, theta_obs, v_obs, w_obs)
    u_star=solution['solution']
    if not solution['exit_status'] == 'Converged':
        pass
        # print('---------------------')
        # print('exit_status', solution['exit_status'])
        # print('num_outer_iterations', solution['num_outer_iterations'])
        # print('f1_infeasibility', solution['f1_infeasibility'])
        # print('last_problem_norm_fpr', solution['last_problem_norm_fpr'])
        # print('penalty', solution['penalty'])

    # print('solve_time_ms', round(solution['solve_time_ms'], 2), 'sek', round(
    #     solution['solve_time_ms']/1000, 2))
    total_sec += solution['solve_time_ms']/1000
    uv=u_star[0:nu*N:2]
    uw=u_star[1:nu*N:2]
    X=[0.0]*(N+1)
    X[0]=x_init
    Y=[0.0]*(N+1)
    Y[0]=y_init
    THETA=[0.0]*(N+1)
    THETA[0]=theta_init
    update_init=True
    for t in range(0, N):
        u_t=u_star[t*nu:(t+1)*nu]
        x=X[t]
        y=Y[t]
        if math.sqrt((x-x_obs)**2) <= r_obs-0.3 and math.sqrt((y-y_obs)**2) <= r_obs-0.3:
            update_init=False
            print('----------')
            print(math.sqrt((x-x_obs)**2))
            print(math.sqrt((y-y_obs)**2))
        theta=THETA[t]
        X[t+1]=x + ts*np.cos(theta)*u_t[0]
        Y[t+1]=y + ts*np.sin(theta)*u_t[0]
        THETA[t+1]=theta + ts*u_t[1]
    curr_dist2ref_x=math.sqrt((X[0]-xref)**2)
    curr_dist2ref_y=math.sqrt((Y[0]-yref)**2)

    avg_progress=math.sqrt((sum(X[1:])/len(X[1:])-xref)**2) < curr_dist2ref_x and math.sqrt(
        (sum(Y[1:])/len(X[1:])-yref)**2) < curr_dist2ref_y
    progress=math.sqrt(
        (X[-1]-xref)**2) < curr_dist2ref_x and math.sqrt((Y[-1]-yref)**2) < curr_dist2ref_y
    if update_init and progress and avg_progress:
        (x_init, y_init, theta_init)=(X[1], Y[1], THETA[1])
        (v_init, w_init)=(uv[0], uw[0])

    ani=animation.FuncAnimation(fig, animate, interval = 100000)
    plt.pause(0.0001)
print('total_sec', total_sec)
plt.show()
mng.kill()
