from opt_build import ts, N, nu, nx
import opengen as og
import numpy as np
import matplotlib.pyplot as plt

mng = og.tcp.OptimizerTcpManager('python_test_build/model_dd_opt')
mng.start()
mng.ping()
# Init ego
(x_init, y_init, theta_init) = (1, 1, np.pi/5)
(xref, yref, thetaref) = (10, 10, 0)
# Init Obstacles
(x_obs,y_obs,theta_obs,r_obs)=(4.5,4.5,0,1)
(v_obs,w_obs)=(0.001,0.00001)

p_lst=[x_init, y_init,theta_init,xref, yref, thetaref]
obs_1=[x_obs,y_obs,theta_obs,v_obs,w_obs,r_obs]
p_lst.extend(obs_1)
solution = mng.call(p_lst, initial_guess=[1.0] * (nu*N))
mng.kill()

# Plot solution
# ------------------------------------
time = np.arange(0, ts*N, ts)
u_star = solution['solution'] 
print('solve_time_ms',solution['solve_time_ms'],'sek',solution['solve_time_ms']/1000)
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

print('v',uv)
print('w',uw)
print(X)
print(Y)

circle1 = plt.Circle((x_obs,y_obs), r_obs, color='r')
fig, ax = plt.subplots()
ax.add_artist(circle1)
ax.set_xlim((x_init-0.5, xref+0.5))
ax.set_ylim((y_init-0.5, yref+0.5))
plt.plot(X, Y, '-o')
plt.show()
