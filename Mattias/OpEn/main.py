from setup import *
from build_opt_traj import len_traj
from build_opt_traj_tilde import len_traj, model_dd_tilde,v_ref
import opengen as og
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation
from camera_find_circles import get_camera_measurements
from Kalman_function import KalmanFilter
import math
import time
from cv2 import cv2
import serial
# # ------ Init Bluetooth
print("Start")
port="COM6" #This will be different for various devices and on windows it will probably be a COM port.
bluetooth=serial.Serial(port, 9600) #Start communications with the bluetooth unit
print("Connected")
bluetooth.flushInput() #This gives the bluetooth a little kick
# -------Init Kalman filter
P_t = np.diag([1.0, 1.0, 1.0])
R_t = np.diag([1.0, 1.0, 1.0])
Q_t = np.diag([1.0, 1.0, 1.0])*30
# ------- Init Camera
cap = cv2.VideoCapture(0)
cam_width=800
cam_hight=800
cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_hight)
# -------Init ego
r=0.5
(xref_orig, yref_orig, thetaref_orig) = (cam_width-50, cam_hight-150, 0) 

Y_REF = [11, 10.99, 10.96, 10.91, 10.84, 10.76, 10.65, 10.53, 10.39, 10.24, 10.08, 9.91, 9.72, 9.53, 9.34, 9.14, 8.94, 8.74, 8.55, 8.35, 8.17, 7.99, 7.82, 7.67, 7.53, 7.4, 7.29, 7.19, 7.12, 7.06, 7.02, 7.0, 7.0, 7.03, 7.07, 7.13, 7.21, 7.3, 7.42, 7.55, 7.69, 7.85, 8.02, 8.2, 8.39, 8.58, 8.78, 8.98, 9.17, 9.37, 9.57, 9.76, 9.94, 10.11, 10.27, 10.42, 10.55, 10.67, 10.77, 10.85, 10.92, 10.97, 10.99]
X_REF = [0.0, 0.16, 0.32, 0.48, 0.65, 0.81, 0.97, 1.13, 1.29, 1.45, 1.61, 1.77, 1.94, 2.1, 2.26, 2.42, 2.58, 2.74, 2.9, 3.06, 3.23, 3.39, 3.55, 3.71, 3.87, 4.03, 4.19, 4.35, 4.52, 4.68, 4.84, 5.0, 5.16, 5.32, 5.48, 5.65, 5.81, 5.97, 6.13, 6.29, 6.45, 6.61, 6.77, 6.94, 7.1, 7.26, 7.42, 7.58, 7.74, 7.9, 8.06, 8.23, 8.39, 8.55, 8.71, 8.87, 9.03, 9.19, 9.35, 9.52, 9.68, 9.84, 10.0]
THETA_REF=[0.0, -0.07841, -0.156035, -0.2321, -0.3058475, -0.37654, -0.44347, -0.5059675, -0.56341, -0.6152225, -0.66089, -0.6999525, -0.7320225, -0.7567775, -0.77397, -0.78343, -0.7850625, -0.7788525, -0.7648575, -0.7432225, -0.71416, -0.6779625, -0.6349925, -0.585675, -0.5305075, -0.47004, -0.404875, -0.3356625, -0.2631, -0.187905, -0.110835, -0.0326575, 0.0458475, 0.1238925, 0.2007025, 0.275505, 0.347555, 0.4161325, 0.4805525, 0.54017, 0.5943925, 0.6426725, 0.684535, 0.719555, 0.7473875, 0.76775, 0.7804425, 0.7853375, 0.782385, 0.7716175, 0.7531375, 0.7271325, 0.6938625, 0.6536625, 0.6069275, 0.55413, 0.495795, 0.4325075, 0.3648975, 0.2936425, 0.2194525, 0.14307, 0.0652575]

Y_REF.extend([Y_REF[-1]]*len_traj*2)
X_REF.extend([X_REF[-1]]*len_traj*2)
# THETA_REF.insert(0,0)
THETA_REF.extend([THETA_REF[-1]]*len_traj*2)
modes=['point','traj','point','point']
mode=last_mode=modes[0]
run_with_tilde=False
# -------Init Obstacles
penalty_margin=0.3
(x_obs, y_obs, theta_obs, r_obs) = (-6, 6, math.pi/2, 0.5)
(v_obs, w_obs) = (0.06, 0.00001)
# -------Init static stuff
r_cone=r_obs+r+penalty_margin
now_obs_pos=(x_obs,y_obs)
last_obs_pos=(x_obs,y_obs)
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
warm_start=True
u_opt = [0.0] * (nu*N)
if mode=='point':
    mng = og.tcp.OptimizerTcpManager('optimizers/ref_point')
    mng.start()
else:
    if run_with_tilde:
        mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj_tilde')
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

def dist_to_ref(x_k, y_k):
    ref_abs_lst = []
    for i in range(len(X_REF)):
        ref_abs_lst.append(
            math.sqrt((X_REF[i]-x_k)**2+(Y_REF[i]-y_k)**2))
    min_val = min(ref_abs_lst)
    idx = ref_abs_lst.index(min_val)
    return idx+1

def calculate_turn_dir(angle_ego,x_k,y_k):
    angle_ego = np.degrees(angle_ego)
    angle_ref = math.degrees(math.atan2(yref-y_k, xref-x_k))  # check this one!
    clws_angle_ref = angle_ref if angle_ref >= 0 else 180+abs(180+angle_ref)
    if abs(clws_angle_ref-angle_ego) >= 180:
        return -dw
    else:
        return dw

def animate(i):
    ax.cla()
    ax.set_xlim((-cam_width*0.1, cam_width+cam_width*0.1))
    ax.set_ylim((-cam_hight*0.1, cam_hight+cam_hight*0.1))
    xs = [x_k*pixels_1M, x_k*pixels_1M + np.cos(theta_k)*50]
    ys = [y_k*pixels_1M,y_k*pixels_1M + np.sin(theta_k)*50]
    ax.plot(X_plot, Y_plot, 'o', color='g', markersize=3)
    ax.plot(plot_x[-1]*pixels_1M, plot_y[-1]*pixels_1M, '-o', color='b', markersize=2)
    if mode=='point':
        ax.add_patch(plt.Circle((end_xref*pixels_1M, end_yref*pixels_1M), 0.3, color='y'))
    else:
        #ax.plot(xtraj, ytraj, '--', color='y',)
        ax.plot(X_REF, Y_REF, '--', color = 'y',markersize=1)
    ax.add_patch(plt.Circle((x_obs, y_obs), r_obs, color='r'))
    ax.add_patch(plt.Circle((x_obs, y_obs), r_obs+r,edgecolor='r',fill=None,linestyle='dashed'))
    ax.add_patch(plt.Circle((x_obs, y_obs), r_cone, edgecolor='r',fill=None,linestyle='dotted'))
    # for i in range(len(X_C)):
    #     ax.add_patch(plt.Circle((X_C[i], Y_C[i]), 0.1, color='b'))
    ax.plot(xs, ys, '-', color='r')

def convert_input(uv,uw):
    threshold=0.1411
    L=0.014
    v_r = (2*uv + L*uw)/(2)
    v_l = (2*uv - L*uw)/(2)
    if abs(v_l)<threshold and abs(v_r)<threshold:
        v_l=x if v_l>v_r else 0
        v_r=x if v_r>v_l else 0
    return v_r,v_l

i=-1
idx_mode=0
plot_x=plot_y=[]
while i<40:
    start=time.time()
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
    just_changed_ref=just_changed_ref-0.5 if just_changed_ref>0 else just_changed_ref
    while True:
        x_cam_pix,y_cam_pix,theta_cam,output,camera_is_ready,pixels_1M = get_camera_measurements(cap)
        cv2.circle(output,(int(xref_orig),int(cam_hight-yref_orig)),4,(0,0,255),10)
        cv2.circle(output,(int(x_cam_pix),int(y_cam_pix)),4,(255,0,0),3)
        cv2.putText(output, str(round(np.degrees(theta_cam),1)), (400, 60),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        y_cam_pix=cam_hight-y_cam_pix
        (x_cam,y_cam)=(x_cam_pix/pixels_1M,y_cam_pix/pixels_1M)
        (xref, yref, thetaref) = (xref_orig/pixels_1M, yref_orig/pixels_1M, thetaref_orig/pixels_1M)
        newMeasurement = np.matrix([x_cam,y_cam,theta_cam]).T
        if camera_is_ready:
            break
        else:
            cv2.imshow("output", output)
        key = cv2.waitKey(1)
        if key == 27:
            break
    if first_cam_measurment:
        (x_km1, y_km1, theta_km1)=(x_cam,y_cam,theta_cam)
    x_model,y_model,theta_model=model_dd(x_km1, y_km1, theta_km1, u_opt[0], u_opt[1]) # Drive
    first_cam_measurment=False
    q_hat_t = np.matrix([[x_model, y_model, theta_model]]).T

    x_k, y_k, theta_k, P_t = KalmanFilter(P_t,R_t,Q_t,q_hat_t,u_opt[0],u_opt[1],newMeasurement,ts)
    (x_km1, y_km1, theta_km1)=(x_k, y_k, theta_k)
    (x_k, y_k, theta_k)=(x_cam,y_cam,theta_cam)
    plot_x.append(x_k)
    plot_y.append(y_k)
    plot_theta.append(theta_k)
    v_obs_est,last_obs_pos=calc_obs_v_est(last_obs_pos,now_obs_pos)
    p_lst = [x_k, y_k, theta_k, u_opt[0], u_opt[1], xref, yref, thetaref]
    obs_1 = [x_obs, y_obs, theta_obs, v_obs_est, w_obs, r_cone]
    p_lst.extend(obs_1)
    u_opt=[0.0] * (nu*N) if not warm_start else u_opt

    if mode=='point':
        if justChanged and last_mode!=mode:
            mng2.kill()
            mng = og.tcp.OptimizerTcpManager('optimizers/ref_point')
            mng.start()
        end_xref = xref
        end_yref = yref
        solution = mng.call(p_lst, initial_guess=u_opt)
    else:
        if justChanged and last_mode!=mode:
            mng.kill()
            if run_with_tilde:
                mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj_tilde')
            else:
                mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj')
            mng2.start()
        idx = dist_to_ref(x_k, y_k)
        xtraj = X_REF[idx:idx+len_traj]
        ytraj = Y_REF[idx:idx+len_traj]
        thetatraj = THETA_REF[idx:idx+len_traj] # TODO
        thetatraj = THETA_REF[i:i+len_traj]
        end_xref = xtraj[-1]
        end_yref = ytraj[-1]
        p_lst.extend(xtraj)
        p_lst.extend(ytraj)
        p_lst.extend(thetatraj)
        solution = mng2.call(p_lst, initial_guess=u_opt)
    justChanged = False
    #----- update obstacle position
    (x_obs, y_obs, theta_obs) = model_dd(x_obs, y_obs, theta_obs, v_obs, w_obs)
    now_obs_pos=(x_obs,y_obs)
    try:
        u_opt = solution['solution']
        # print(solution['exit_status'])
    except:
        print('No Solution')
        continue
    total_sec += solution['solve_time_ms']/1000
    #------- calc green traj
    uv = u_opt[0:nu*N:2]
    uw = u_opt[1:nu*N:2]
    X = [0.0]*(N+1)
    X_plot= [0.0]*(N+1)
    X[0] = x_k
    X_plot[0]=x_k*pixels_1M
    Y = [0.0]*(N+1)
    Y[0] = y_k
    Y_plot= [0.0]*(N+1)
    Y_plot[0]=y_k*pixels_1M
    THETA = [0.0]*(N+1)
    THETA[0] = theta_k
    collision = False
    x_obs_future=x_obs
    y_obs_future=y_obs
    (X_C,Y_C)=([],[])
    for t in range(0, N):
        u_t = u_opt[t*nu:(t+1)*nu]
        x = X[t]
        y = Y[t]
        theta = THETA[t]
        X[t+1] = x + ts*np.cos(theta)*u_t[0]#+np.random.randn(1)[0]/100
        X_plot.append(X[t+1]*pixels_1M)
        Y[t+1] = y + ts*np.sin(theta)*u_t[0]#+np.random.randn(1)[0]/100
        Y_plot.append(Y[t+1]*pixels_1M)
        THETA[t+1] = theta + ts*u_t[1]
        x_obs_future=x_obs_future + ts*np.cos(theta_obs)*v_obs_est
        y_obs_future=y_obs_future + ts*np.sin(theta_obs)*v_obs_est
        get_cone_const(X[t+1],Y[t+1],THETA[t+1],x_obs_future,y_obs_future,u_t[0])
        if np.sqrt(np.power(X[t+1]-x_obs_future,2)+np.power(Y[t+1]-y_obs_future,2)) <= (r_obs+r):
            collision = True
        if run_with_tilde:
            x, y, theta,x_tild, y_tild, theta_tild = model_dd_tilde(X[t+1], Y[t+1], THETA[t+1], u_t[0]-v_ref, u_t[1],xtraj[t],ytraj[t],thetatraj[t],u_t[0])
    curr_dist2ref = math.sqrt((x_k-end_xref)**2+(y_k-end_yref)**2)
    avg_progress = math.sqrt((sum(X[1:])/len(X[1:])-end_xref)**2+(sum(Y[1:])/len(X[1:])-end_yref)**2) < (curr_dist2ref+just_changed_ref)
    progress = math.sqrt((X[-1]-end_xref)**2+ (Y[-1]-end_yref)**2) < (curr_dist2ref+just_changed_ref)
    close_to_target = math.sqrt((X[0]-end_xref)**2+(Y[0]-end_yref)**2) < 0.2 # TUNING
    if  collision or not progress or not avg_progress or close_to_target: # Decide if to use u_opt
        u_opt = [0.0] * (nu*N)
    if not close_to_target and not progress and not collision: #TODO
        u_opt = [0.0] * (nu*N)
        u_opt[1] = calculate_turn_dir(theta_k,x_k,y_k)  # set every u_opt[1] to u_opt[1:nu*N:2]
    v_r, v_l=convert_input(u_opt[0],u_opt[1])
    # print('u:',round(u_opt[0],3), round(u_opt[1],3))
    print('vr',round(v_r,3),'vl', round(v_l,3))
    bluetooth.write(b"<"+str.encode(str(v_r))+ b"," + str.encode(str(v_l)) + b">") #These need to be bytes not unicode, plus a number
    # input_data=bluetooth.readline() #This reads the incoming data. In this particular example it will be the "Hello from Blue" line
    # print(input_data.decode())  # Plot what the arduino board is getting. Incoming in bytes, thus decode
    ani = animation.FuncAnimation(fig, animate, interval=100000)
    plt.pause(0.00002)
    print('Time Passed',round(time.time()-start,3))
    key = cv2.waitKey(1)
    if key == 27:
        break
    print(i)
bluetooth.write(b"<"+str.encode(str(0))+ b"," + str.encode(str(0)) + b">")
print('total_sec', total_sec)
print('avrage time_ms',total_sec*1000/i)
plt.show()
try:
    mng2.kill()
except:
    mng.kill()
cap.release()
cv2.destroyAllWindows()