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

print("Start")
port="COM13" #This will be different for various devices and on windows it will probably be a COM port.
bluetooth=serial.Serial(port, 9600) #Start communications with the bluetooth unit
print("Connected")
bluetooth.flushInput() #This gives the bluetooth a little kick

# -------Init ego
(x_init, y_init, theta_init,r) = (0, 11, np.pi/1000,0.5)
(xref, yref, thetaref) = (10, 10, 0)
(v_init, w_init) = (0, 0)
Y_REF = [11, 10.99, 10.96, 10.91, 10.84, 10.76, 10.65, 10.53, 10.39, 10.24, 10.08, 9.91, 9.72, 9.53, 9.34, 9.14, 8.94, 8.74, 8.55, 8.35, 8.17, 7.99, 7.82, 7.67, 7.53, 7.4, 7.29, 7.19, 7.12, 7.06, 7.02, 7.0, 7.0, 7.03, 7.07, 7.13, 7.21, 7.3, 7.42, 7.55, 7.69, 7.85, 8.02, 8.2, 8.39, 8.58, 8.78, 8.98, 9.17, 9.37, 9.57, 9.76, 9.94, 10.11, 10.27, 10.42, 10.55, 10.67, 10.77, 10.85, 10.92, 10.97, 10.99]
X_REF = [0.0, 0.16, 0.32, 0.48, 0.65, 0.81, 0.97, 1.13, 1.29, 1.45, 1.61, 1.77, 1.94, 2.1, 2.26, 2.42, 2.58, 2.74, 2.9, 3.06, 3.23, 3.39, 3.55, 3.71, 3.87, 4.03, 4.19, 4.35, 4.52, 4.68, 4.84, 5.0, 5.16, 5.32, 5.48, 5.65, 5.81, 5.97, 6.13, 6.29, 6.45, 6.61, 6.77, 6.94, 7.1, 7.26, 7.42, 7.58, 7.74, 7.9, 8.06, 8.23, 8.39, 8.55, 8.71, 8.87, 9.03, 9.19, 9.35, 9.52, 9.68, 9.84, 10.0]
THETA_REF=[0, -0.1996668, -0.3973387, -0.5910404, -0.7788367, -0.9588511, -1.1292849, -1.2884354, -1.4347122, -1.5666538, -1.682942, -1.7824147, -1.8640782, -1.9271164, -1.9708995, -1.99499, -1.9991472, -1.9833296, -1.9476953, -1.8926002, -1.8185949, -1.7264187, -1.6169928, -1.4914104, -1.3509264, -1.1969443, -1.0310027, -0.8547598, -0.6699763, -0.4784987, -0.28224, -0.0831613, 0.1167483, 0.3154914, 0.5110822, 0.7015665, 0.8850409, 1.0596723, 1.2237158, 1.3755323, 1.513605, 1.6365542, 1.7431515, 1.8323319, 1.9032041, 1.9550602, 1.987382, 1.9998465, 1.9923292, 1.9649052, 1.9178485, 1.8516294, 1.7669093, 1.6645349, 1.545529, 1.4110807, 1.2625333, 1.1013711, 0.9292044, 0.7477533, 0.558831, 0.364325, 0.1661788]

Y_REF.extend([Y_REF[-1]]*len_traj*2)
X_REF.extend([X_REF[-1]]*len_traj*2)
THETA_REF.insert(0,0)
THETA_REF.extend([THETA_REF[-1]]*len_traj*2)
modes=['traj','point','point']
mode=modes[0]
# -------Init Obstacles
penalty_margin=0.3
(x_obs, y_obs, theta_obs, r_obs) = (-6, 8, math.pi/2, 0.5)
(v_obs, w_obs) = (0.1, 0.00001)
# -------Init static stuff
r_cone=r_obs+r+penalty_margin
now_obs_pos=(x_obs,y_obs)
last_obs_pos=(x_obs,y_obs)
plot_init_x = x_init
plot_init_y = y_init
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
warm_start=False
u_star = [0.0] * (nu*N)
if mode=='point':
    mng = og.tcp.OptimizerTcpManager('optimizers/ref_point')
    mng.start()
else:
    mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj')
    mng2.start()

def get_cone_const(x,y,theta,x_obs,y_obs,uk):
    rterm = (x-x_obs)**2+(y-y_obs)**2
    uterm = ((cs.cos(theta)*uk-v_obs)*(x-x_obs) +  (cs.sin(theta)*uk-v_obs)*(y-y_obs))**2
    lterm = (cs.cos(theta)*uk-v_obs)**2+(cs.sin(theta)*uk-v_obs)**2
    # d=rterm-(uterm/lterm)
    # r=np.sqrt((x-x_obs)**2+(y-y_obs)**2)
    vab=np.sqrt(uterm/lterm)
    x_c=np.cos(theta)*vab
    y_c=np.sin(theta)*vab
    # print('d:',np.sqrt(d),'Const:',np.sqrt(d)>=r_obs+r)
    X_C.append(x_c+x)
    Y_C.append(y_c+y)


def calc_obs_v_est(last_obs_pos,now_obs_pos):
    global last_time
    dist=np.sqrt(np.power(last_obs_pos[0]-now_obs_pos[0],2)+np.power(last_obs_pos[1]-now_obs_pos[1],2))
    dt=time.time()-last_time
    last_time=time.time()
    v_obs_est=dist/ts  # byt ut mot dt senare
    last_obs_pos=now_obs_pos
    return v_obs_est,last_obs_pos

def dist_to_ref(x_init, y_init):
    ref_abs_lst = []
    for i in range(len(X_REF)):
        ref_abs_lst.append(
            math.sqrt((X_REF[i]-x_init)**2+(Y_REF[i]-y_init)**2))
    min_val = min(ref_abs_lst)
    idx = ref_abs_lst.index(min_val)
    return idx+1


def calculate_turn_dir(angle_ego):
    angle_ego = np.degrees(angle_ego)
    angle_ref = math.degrees(math.atan2(
        yref-y_init, xref-x_init))  # check this one!
    clws_angle_ref = angle_ref if angle_ref >= 0 else 180+abs(180+angle_ref)
    if abs(clws_angle_ref-angle_ego) >= 180:
        return -ts*dw
    else:
        return ts*dw


def animate(i):
    ax.cla()
    ax.set_xlim((-0.5, 12))
    ax.set_ylim((-0.5, 12))
    xs = [X[0], X[0] + np.cos(theta_init)*0.3]
    ys = [Y[0], Y[0] + np.sin(theta_init)*0.3]
    ax.plot(X, Y, 'o', color='g', markersize=3)
    ax.plot(plot_x[-1], plot_y[-1], '-o', color='b', markersize=2)
    if mode=='point':
        ax.add_patch(plt.Circle((end_xref, end_yref), 0.3, color='y'))
    else:
        ax.plot(xtraj, ytraj, '--', color='y',)
        #ax.plot(X_REF, Y_REF, '--', color = 'y',markersize=1)
    ax.add_patch(plt.Circle((x_obs, y_obs), r_obs, color='r'))
    ax.add_patch(plt.Circle((x_obs, y_obs), r_obs+r,edgecolor='r',fill=None,linestyle='dashed'))
    ax.add_patch(plt.Circle((x_obs, y_obs), r_cone, edgecolor='r',fill=None,linestyle='dotted'))
    for i in range(len(X_C)):
        ax.add_patch(plt.Circle((X_C[i], Y_C[i]), 0.1, color='b'))
    ax.plot(xs, ys, '-', color='r')

def convert_input(uv,uw):
    # 2/0.07 =~ 28,57143
    v_r = 28.57143*uv + 28.57143*uw
    v_l = 28.57143*uv - 28.57143*uw
    return v_r,v_l


i=-1
idx_mode=0
while i<100:
    i+=1
    if close_to_target:
        break
        last_mode=mode
        plot_x=[plot_x[-1]]
        plot_y=[plot_y[-1]] # curr pos
        idx_mode=idx_mode+1 if not idx_mode==len(modes) else 0
        mode=modes[idx_mode]
        just_changed_ref = 5
        justChanged = True  
        if idx_mode==0:
            (x_obs, y_obs, theta_obs, r_obs) = (-10, -10, 0, 0.5) 
            (v_obs, w_obs) = (0.0, 0.00001)   
        elif idx_mode==1:
            (xref, yref, thetaref) = (9, 1, 0)
            (x_obs, y_obs, theta_obs, r_obs) = (9, 5, 0, 0.5)
            (v_obs, w_obs) = (-0.1, 0.00001)
        else:
            (xref, yref, thetaref) = (0, 9.5, 0)
            (x_obs, y_obs, theta_obs, r_obs) = (6, 6, 5*math.pi/4, 0.5)
            (v_obs, w_obs) = (0.1, 0.00001)
        now_obs_pos=(x_obs,y_obs)
        last_obs_pos=(x_obs,y_obs)

    v_obs_est,last_obs_pos=calc_obs_v_est(last_obs_pos,now_obs_pos)
    plot_x.append(x_init)
    plot_y.append(y_init)
    plot_theta.append(theta_init)
    p_lst = [x_init, y_init, theta_init, v_init, w_init, xref, yref, thetaref]
    obs_1 = [x_obs, y_obs, theta_obs, v_obs_est, w_obs, r_cone]
    p_lst.extend(obs_1)
    u_star=[0.0] * (nu*N) if not warm_start else u_star
    if mode=='point':
        if justChanged and last_mode!=mode:
            mng2.kill()
            mng = og.tcp.OptimizerTcpManager('optimizers/ref_point')
            mng.start()
        end_xref = xref
        end_yref = yref
        solution = mng.call(p_lst, initial_guess=u_star)
    else:
        if justChanged and last_mode!=mode:
            mng.kill()
            mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj')
            mng2.start()
        idx = dist_to_ref(x_init, y_init)
        xtraj = X_REF[idx:idx+len_traj]
        ytraj = Y_REF[idx:idx+len_traj]
        thetatraj = THETA_REF[idx:idx+len_traj]
        end_xref = xtraj[-1]
        end_yref = ytraj[-1]
        p_lst.extend(xtraj)
        p_lst.extend(ytraj)
        p_lst.extend(thetatraj)
        solution = mng2.call(p_lst, initial_guess=u_star)
    if just_changed_ref > 0:
        just_changed_ref -= 0.5
    justChanged = False
    (x_obs, y_obs, theta_obs) = model_dd(x_obs, y_obs, theta_obs, v_obs, w_obs)
    now_obs_pos=(x_obs,y_obs)
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
    x_obs_future=x_obs
    y_obs_future=y_obs
    (X_C,Y_C)=([],[])
    for t in range(0, N):
        u_t = u_star[t*nu:(t+1)*nu]
        x = X[t]
        y = Y[t]
        theta = THETA[t]
        X[t+1] = x + ts*np.cos(theta)*u_t[0]#+np.random.randn(1)[0]/100
        Y[t+1] = y + ts*np.sin(theta)*u_t[0]#+np.random.randn(1)[0]/100 
        THETA[t+1] = theta + ts*u_t[1]
        x_obs_future=x_obs_future + ts*np.cos(theta_obs)*v_obs_est
        y_obs_future=y_obs_future + ts*np.sin(theta_obs)*v_obs_est
        get_cone_const(X[t+1],Y[t+1],THETA[t+1],x_obs_future,y_obs_future,u_t[0])
        if np.sqrt(np.power(X[t+1]-x_obs_future,2)+np.power(Y[t+1]-y_obs_future,2)) <= (r_obs+r):
            collision = True
    curr_dist2ref = math.sqrt((X[0]-end_xref)**2+(Y[0]-end_yref)**2)
    avg_progress = math.sqrt((sum(X[1:])/len(X[1:])-end_xref)**2+(sum(Y[1:])/len(X[1:])-end_yref)**2) < (curr_dist2ref+just_changed_ref)
    progress = math.sqrt((X[-1]-end_xref)**2+ (Y[-1]-end_yref)**2) < (curr_dist2ref+just_changed_ref)
    close_to_target = math.sqrt((X[0]-end_xref)**2+(Y[0]-end_yref)**2) < 0.2
    close_to_target=False
    if not collision and progress and avg_progress and not close_to_target:
        (x_init, y_init, theta_init) = (X[1], Y[1], THETA[1])
        (v_init, w_init) = (uv[0], uw[0])

    if not close_to_target and not progress and not collision and False:
        theta_init += calculate_turn_dir(theta_init)    
    # Send inputsignal to arduino:
    (v_r,v_l) = convert_input(uv[0],uw[0])
    #print(v_r)
    #print('---------')
    #print(v_l)
    bluetooth.write(b"<"+str.encode(str(v_r))+ b"," + str.encode(str(v_l)) + b">")#These need to be bytes not unicode, plus a number
    input_data=bluetooth.readline()#This reads the incoming data. In this particular example it will be the "Hello from Blue" line
    print(input_data.decode())  # Plot what the arduino board is getting. Incoming in bytes, thus decode
    
    ani = animation.FuncAnimation(fig, animate, interval=100000)
    plt.pause(0.1)

print('total_sec', total_sec)
print('avrage time_ms',total_sec*1000/i)
plt.show()
try:
    mng2.kill()
except:
    mng.kill()




