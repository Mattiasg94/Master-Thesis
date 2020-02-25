import opengen as og
import casadi.casadi as cs
import numpy as np

ts = 0.5
(nu, nx, nref, N) = (2, 3, 3, 22)
nObs = (6)
(Qx, Qy, Qtheta) = (10, 10, 0)
(Rv, Rw) = (1, 1)
(Qtx, Qty, Qttheta) = (100, 100, 0)
(vmin, vmax) = (0, 1)
(wmin, wmax) = (-1, 1)


def model_dd(x, y, theta, v, w):
    x += ts*cs.cos(theta)*v
    y += ts*cs.sin(theta)*v
    theta += ts*w
    return x, y, theta


def build_opt():
    u = cs.SX.sym('u', nu*N)
    p = cs.SX.sym('p', nx+nref+nObs)
    (x, y, theta) = (p[0], p[1], p[2])
    (xref, yref, thetaref) = (p[3], p[4], p[5])
    (x_obs, y_obs, theta_obs, v_obs, w_obs, r_obs) = (
        p[6], p[7], p[8], p[9], p[10], p[11])

    cost = 0
    c = 0
    # -------Collission constrint
    for t in range(0, nu*N, nu): 
        u_t = u[t:t+2]
        cost += Qx*(x-xref)**2 + Qy*(y-yref)**2 + Qtheta*(theta-thetaref)**2
        cost += Rv*u_t[0]**2+Rw*u_t[1]**2
        (x, y, theta) = model_dd(x, y, theta, u_t[0], u_t[1])
        (x_obs, y_obs, theta_obs) = model_dd(x_obs, y_obs, theta_obs, v_obs, w_obs)
        rterm = (x-x_obs)**2+(y-y_obs)**2
        uterm = (cs.cos(theta)*u_t[0]-v_obs)*(x-x_obs)+(cs.sin(theta)*u_t[0]-v_obs)*(y-y_obs)
        lterm = (cs.cos(theta)*u_t[0]-v_obs)**2+(cs.sin(theta)*u_t[0]-v_obs)**2        
        # -------distance const
        #c += cs.fmax(0.0, r_obs - (x_obs-x)**2 - (y_obs-y)**2)
        # -------regular cone
        #c += cs.fmax(0.0, r_obs**2*lterm-(rterm*lterm-uterm**2))
        # -------cone only when dist ego > dist obs
        cone = r_obs**2*lterm-(rterm*lterm-uterm**2)
        ego_dist = cs.sqrt((x-xref)**2+(y-yref)**2)
        obs_dist = cs.sqrt((x_obs-xref)**2+(y_obs-yref)**2)
        c += cs.fmax(0.0, -(obs_dist-ego_dist)) * (cs.fmax(0.0, cone))

    cost += Qtx*(x-xref)**2 + Qty*(y-yref)**2 + Qttheta*(theta-thetaref)**2
    # -------Acceleration constraint
    (dv_c, dw_c) = (0, 0)
    u_tm1 = [0, 0]
    for t in range(0, nu*N, nu):
        u_t = u[t:t+2]
        dv_c += cs.fmax(0.0, u_t[0]-u_tm1[0]-0.2)
        dw_c += cs.vertcat(cs.fmax(0.0, u_t[1]-u_tm1[1]-0.05),
                           cs.fmax(0.0, u_tm1[1]-u_t[1]-0.05))
        u_tm1 = u_t
    # -------Boundery constrint
    umin = []
    umax = []
    for i in range(N):
        umin.extend([vmin, wmin])
        umax.extend([vmax, wmax])

    bounds = og.constraints.Rectangle(umin, umax)
    C = cs.vertcat(c, dv_c, dw_c)
    problem = og.builder.Problem(u, p, cost).with_penalty_constraints(
        C).with_constraints(bounds)

    build_config = og.config.BuildConfiguration()\
        .with_build_directory("python_test_build")\
        .with_build_mode("debug")\
        .with_tcp_interface_config()

    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("model_dd_opt")

    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(1e-5)\
        .with_initial_tolerance(1e-5)\
        .with_max_outer_iterations(5)\
        .with_delta_tolerance(1e-4)\
        .with_penalty_weight_update_factor(10.0).with_initial_penalty(10)

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                              meta,
                                              build_config,
                                              solver_config) \
        .with_verbosity_level(1)

    builder.build()


if __name__ == '__main__':
    build_opt()
    import run_opt
