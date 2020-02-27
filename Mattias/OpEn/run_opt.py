from opt_build_point import ts, N, nu, nx, model_dd, dw
from opt_build_point import len_traj
import opengen as og
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation
import math

# -------Init ego
(x_init, y_init, theta_init) = (0, 0, np.pi/4)
(xref, yref, thetaref,r) = (10, 10, 0,0.5)
(v_init, w_init) = (0, 0)
ref_point = False
Y_REF = [0, 1.9, 4.2, 6.1, 7.7, 8.9, 9.6, 10, 10, 10, 10, 10, 10]
X_REF = [0, 1.0, 2.1, 3.6, 5.5, 7.5, 8.9, 10, 10, 10, 10, 10, 10]
# -------Init Obstacles
penalty_margin=0.3
(x_obs, y_obs, theta_obs, r_obs) = (3, 5, 0, 0.5)
(v_obs, w_obs) = (0.0, 0.00001)
# -------Init static stuff
r_cone=r_obs+r+penalty_margin
plot_x = []
plot_y = []
plot_theta = []
plot_init_x = x_init
plot_init_y = y_init
total_sec = 0
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
just_changed_ref = 0
justChanged = False
u_star = [0.0] * (nu*N)
if ref_point:
    mng = og.tcp.OptimizerTcpManager('optimizers/ref_point')
    mng.start()
else:
    mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj')
    mng2.start()


def dist_to_ref(x_init, y_init):
    ref_abs_lst = []
    for i in range(len(X_REF)):
        ref_abs_lst.append(
            math.sqrt((X_REF[i]-x_init)**2+(Y_REF[i]-y_init)**2))
    min_val = min(ref_abs_lst)
    idx = ref_abs_lst.index(min_val)
    return idx


def calculate_turn_dir(angle_ego):
    angle_ego = np.degrees(angle_ego)
    angle_ref = math.degrees(math.atan2(
        yref-y_init, xref-xref))  # check this one!
    clws_angle_ref = angle_ref if angle_ref >= 0 else 180+abs(180+angle_ref)
    if abs(clws_angle_ref-angle_ego) >= 180:
        return -ts*dw
    else:
        return ts*dw


def animate(i):
    ax.cla()
    ax.set_xlim((-0.5, 10.5))
    ax.set_ylim((-0.5, 10.5))
    xs = [x_init, x_init + np.cos(theta_init)*0.3]
    ys = [y_init, y_init + np.sin(theta_init)*0.3]
    ax.plot(X, Y, 'o', color='g', markersize=2)
    ax.plot(plot_x[-1], plot_y[-1], '-o', color='b', markersize=2)
    if ref_point:
        ax.add_patch(plt.Circle((end_xref, end_yref), 0.3, color='y'))
    else:
        ax.plot(xtraj, ytraj, '--', color='y',)
    #ax.plot(X_REF, Y_REF, '--', color = 'y',)
    ax.add_patch(plt.Circle((x_obs, y_obs), r_obs, color='r'))
    ax.add_patch(plt.Circle((x_obs, y_obs), r_obs+r,edgecolor='r',fill=None,linestyle='dashed'))
    ax.add_patch(plt.Circle((x_obs, y_obs), r_cone, edgecolor='r',fill=None,linestyle='dotted'))
    ax.plot(xs, ys, '-', color='r')


for i in range(160):
    # if i == 30 or i == 80:
    #     ref_point = not ref_point
    #     just_changed_ref = 5
    #     justChanged = True
    plot_x.append(x_init)
    plot_y.append(y_init)
    plot_theta.append(theta_init)
    p_lst = [x_init, y_init, theta_init, v_init, w_init, xref, yref, thetaref]
    obs_1 = [x_obs, y_obs, theta_obs, v_obs, w_obs, r_cone]
    p_lst.extend(obs_1)
    if ref_point:
        if justChanged:
            mng2.kill()
            mng = og.tcp.OptimizerTcpManager('optimizers/ref_point')
            mng.start()
        end_xref = xref
        end_yref = yref
        solution = mng.call(p_lst, initial_guess=u_star)
    else:
        if justChanged:
            mng.kill()
            mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj')
            mng2.start()
        idx = dist_to_ref(x_init, y_init)+1
        xtraj = X_REF[idx:idx+len_traj]
        ytraj = Y_REF[idx:idx+len_traj]
        end_xref = xtraj[-1]
        end_yref = ytraj[-1]
        p_lst.extend(xtraj)
        p_lst.extend(ytraj)
        solution = mng2.call(p_lst, initial_guess=u_star)
    if just_changed_ref > 0:
        just_changed_ref -= 0.5
    justChanged = False
    #u_star=[0.0] * (nu*N)
    (x_obs, y_obs, theta_obs) = model_dd(x_obs, y_obs, theta_obs, v_obs, w_obs)
    try:
        u_star = solution['solution']
    except:
        print('No Solution')
        continue
    if not solution['exit_status'] == 'Converged':
        pass
        # print('---------------------')
        # print('exit_status', solution['exit_status'])
        # print('num_outer_iterations', solution['num_outer_iterations'])
        # # print('f1_infeasibility', solution['f1_infeasibility'])
        # # print('last_problem_norm_fpr', solution['last_problem_norm_fpr'])
        # print('penalty', solution['penalty'])
        # # print('solve_time_ms', round(solution['solve_time_ms'], 2), 'sek', round(
        # #     solution['solve_time_ms']/1000, 2))
    total_sec += solution['solve_time_ms']/1000
    uv = u_star[0:nu*N:2]
    uw = u_star[1:nu*N:2]
    X = [0.0]*(N+1)
    X[0] = x_init
    Y = [0.0]*(N+1)
    Y[0] = y_init
    THETA = [0.0]*(N+1)
    THETA[0] = theta_init
    collision = False
    for t in range(0, N):
        u_t = u_star[t*nu:(t+1)*nu]
        x = X[t]
        y = Y[t]
        theta = THETA[t]
        X[t+1] = x + ts*np.cos(theta)*u_t[0]
        Y[t+1] = y + ts*np.sin(theta)*u_t[0]
        THETA[t+1] = theta + ts*u_t[1]
        if np.sqrt(np.power(X[t+1]-x_obs,2)) <= (r_obs+r) and np.sqrt(np.power(Y[t+1]-y_obs,2)) <= (r_obs+r):
            collision = True
            print('--------------')
            print(np.sqrt(np.power(X[t+1]-x_obs,2)),r_obs+r)
            print(np.sqrt(np.power(Y[t+1]-y_obs,2)) ,r_obs+r)
    curr_dist2ref_x = math.sqrt((X[0]-end_xref)**2)
    curr_dist2ref_y = math.sqrt((Y[0]-end_yref)**2)

    avg_progress = math.sqrt((sum(X[1:])/len(X[1:])-end_xref)**2) < (curr_dist2ref_x+just_changed_ref) or math.sqrt(
        (sum(Y[1:])/len(X[1:])-end_yref)**2) < (curr_dist2ref_y+just_changed_ref)
    progress = math.sqrt((X[-1]-end_xref)**2) < (curr_dist2ref_x+just_changed_ref) or math.sqrt(
        (Y[-1]-end_yref)**2) < (curr_dist2ref_y+just_changed_ref)
    close_to_target = math.sqrt(
        (X[0]-end_xref)**2) < 0.2 and math.sqrt((Y[0]-end_yref)**2) < 0.2
    if not collision and progress and avg_progress and not close_to_target:
        (x_init, y_init, theta_init) = (X[1], Y[1], THETA[1])
        (v_init, w_init) = (uv[0], uw[0])
    else:
        if not close_to_target and not collision:
            print('close_to_target:', close_to_target, 'collision:',
                  collision, 'progress:', progress, 'avg_progress:', avg_progress)
        theta_init += calculate_turn_dir(theta_init)
    if not close_to_target and not progress and not collision:
        theta_init += calculate_turn_dir(theta_init)
    ani = animation.FuncAnimation(fig, animate, interval=100000)
    plt.pause(0.1)
    break
print('total_sec', total_sec)
plt.show()
try:
    mng2.kill()
except:
    mng.kill()
