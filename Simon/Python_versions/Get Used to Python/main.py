import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

# Build parametric optimizer
# ------------------------------------

build_optimizer = 1
if build_optimizer == 1:
    # define variables
    (nu, nx, N) = (2, 3, 15)
    L = 0.5
    ts = 0.1
    (xref , yref, thetaref, wref) = (4, 4, 4, 0)
    (Qxy, Qtheta,Rv, Rw, qN, qthetaN) = (10, 0.1, 1, 1, 200, 2)

    # Define symbolic
    u = cs.SX.sym('u', nu*N)
    z0 = cs.SX.sym('z0', nx)

    # Define initial values. Note that Z0 should be supplied iteratively 
    (x, y, theta) = (z0[0], z0[1], z0[2])
    cost = 0

    # Build cost function
    for t in range(0, nu*N, nu):
        # State cost
        cost += Qxy*((x-xref)**2 + (y-yref)**2) + Qtheta*(theta-thetaref)**2

        # Input cost
        u_t = u[t:t+2]
        cost += Rv*u_t[0] + Rw*u_t[1]

        # Model
        x += x + ts*u_t[0]*cs.cos(theta) #ts * (u_t[0] + L * cs.sin(theta) * theta_dot)
        y += y + ts*u_t[0]*cs.sin(theta) #ts * (u_t[1] - L * cs.cos(theta) * theta_dot)
        theta += theta + ts*u_t[1]

    # What is qN here?? (qN = 200). (qthetaN = 2)
    cost += qN*((x-xref)**2 + (y-yref)**2) + qthetaN*(theta-thetaref)**2

    # Constrain input, LB/UB
    umin = [-1.0] * (nu*N)
    umax = [1.0] * (nu*N)
    bounds = og.constraints.Rectangle(umin, umax)

    problem = og.builder.Problem(u, z0, cost).with_constraints(bounds)
    build_config = og.config.BuildConfiguration()\
        .with_build_directory("model_DD")\
        .with_build_mode("debug")\
        .with_tcp_interface_config()
    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("navigation")
    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(1e-5)
    builder = og.builder.OpEnOptimizerBuilder(problem, 
                                            meta,
                                            build_config, 
                                            solver_config) \
        .with_verbosity_level(1)
    builder.build()
    print('Build finished')

# Use TCP server
# ------------------------------------
mng = og.tcp.OptimizerTcpManager('model_DD/navigation')
mng.start()
mng.ping()
Nsim = 20
z0 = np.zeros(4)


for k in range(0, Nsim, 1):
    #    x(k+1,:) =A*x(k,:)' + B*u(k,:)'    
    solution = mng.call(z0[0:2], initial_guess=z0[3:4])  
    

mng.kill()
# Plot solution
# ------------------------------------
time = np.arange(0, ts*N, ts)
u_star = solution  #['solution']
ux = u_star[0:nu*N:2]
uy = u_star[1:nu*N:2]

plt.subplot(211)
plt.plot(time, ux, '-o')
plt.ylabel('u_x')
plt.subplot(212)
plt.plot(time, uy, '-o')
plt.ylabel('u_y')
plt.xlabel('Time')
plt.show()