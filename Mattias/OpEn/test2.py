from opt_build import ts, N, nu, nx
import opengen as og
import numpy as np
import matplotlib.pyplot as plt

mng = og.tcp.OptimizerTcpManager('python_test_build/model_dd_opt')
mng.start()

mng.ping()
(x_init, y_init, theta_init) = (-1, -1, np.pi/4)
solution = mng.call([x_init, y_init, theta_init], initial_guess=[1.0] * (nu*N))
mng.kill()

# Plot solution
# ------------------------------------
time = np.arange(0, ts*N, ts)
u_star = solution['solution']
ux = u_star[0:nu*N:2]
uy = u_star[1:nu*N:2]
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

# print(x_states)
print(X)
print(Y)

circle1 = plt.Circle((0, 0), 0.5, color='r')
fig, ax = plt.subplots()
ax.add_artist(circle1)
ax.set_xlim((x_init, 2.5))
ax.set_ylim((y_init, 2.5))
plt.plot(X, Y, '-o')
plt.show()
