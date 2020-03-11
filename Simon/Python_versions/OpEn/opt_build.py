import opengen as og
import casadi.casadi as cs
import numpy as np

ts = 0.5
(nu, nx, nref, N) = (2, 3, 3, 15)
ntraj = 6
nObs = (6)
num_obst = 3 # Probably have to be adapted to multiple different scenarios
(Qx, Qy, Qtheta) = (10, 10, 0)
(Rv, Rw) = (0, 0)
(Qtx, Qty, Qttheta) = (100, 100, 0)
(vmin, vmax) = (0, 1)
(wmin, wmax) = (-1, 1)

def model_dd(x, y, theta, v, w):
    L = 0.15
    x += ts*cs.cos(theta)*v#+cs.cos(theta)*L # Construct offset to get vehicle center.
    y += ts*cs.sin(theta)*v#+cs.sin(theta)*L # Construct offset to get vehicle center.
    theta += ts*w
    return x, y, theta

def model_dd_obst(x, y, theta, v, w):
    x += ts*cs.cos(theta)*v#+cs.cos(theta)*L # Construct offset to get vehicle center.
    y += ts*cs.sin(theta)*v#+cs.sin(theta)*L # Construct offset to get vehicle center.
    theta += ts*w
    return x, y, theta

def build_opt():
    u = cs.SX.sym('u', nu*N)
    p = cs.SX.sym('p', nx+nu+nref+nObs*num_obst+ntraj)  
    (x, y, theta) = (p[0], p[1], p[2])
    ukm1=[p[3],p[4]]
    (xref, yref, thetaref) = (p[5], p[6], p[7])
    k = 0
    x_obs = []
    y_obs = []
    theta_obs = []
    v_obs = []
    w_obs = []
    r_obs = []
    for k in range(8,nx+nu+nref+nObs*num_obst,6):
        x_obs.append(p[k])      
        y_obs.append(p[k+1])   
        theta_obs.append(p[k+2])
        r_obs.append(p[k+3])
        v_obs.append(p[k+4])      
        w_obs.append(p[k+5])  
        num_vars = k   
    xtraj=[p[num_vars+6],p[num_vars+7],p[num_vars+8]]
    ytraj=[p[num_vars+9],p[num_vars+10],p[num_vars+11]]        
    cost = 0
    c = 0
    # -------Collission constrint
    k=0
    for j,t in enumerate(range(0, nu*N, nu)):
        uk = u[t:t+2]
        # -------Reference trajectory
        cost += Qx*(x-xtraj[k])**2 + Qy*(y-ytraj[k])**2 + Qtheta*(theta-thetaref)**2
        if j%len(xtraj)==0 and not t==0:
            k+=1
        # -------Reference Point
        # cost += Qx*(x-xref)**2 + Qy*(y-yref)**2 + Qtheta*(theta-thetaref)**2
        # -------
        cost += Rv*uk[0]**2+Rw*uk[1]**2
        (x, y, theta) = model_dd(x, y, theta, uk[0], uk[1])
        for k in range(num_obst):
            (x_obs[k], y_obs[k], theta_obs[k]) = model_dd(
                x_obs[k], y_obs[k], theta_obs[k], v_obs[k], w_obs[k])

            rterm = (x-x_obs[k])**2+(y-y_obs[k])**2
            uterm = (cs.cos(theta)*uk[0]-v_obs[k])*(x-x_obs[k]) + \
                (cs.sin(theta)*uk[0]-v_obs[k])*(y-y_obs[k])
            lterm = (cs.cos(theta)*uk[0]-v_obs[k])**2+(cs.sin(theta)*uk[0]-v_obs[k])**2
            # -------distance const
            # c += cs.fmax(0.0, r_obs[k] - (x_obs[k]-x)**2 - (y_obs[k]-y)**2)
            # -------regular cone
            c += cs.fmax(0.0, r_obs[k]**2*lterm-(rterm*lterm-uterm**2))
            # -------cone only when dist ego > dist obs
            # cone = r_obs[k]**2*lterm-(rterm*lterm-uterm**2)
            # ego_dist = cs.sqrt((x-xref)**2+(y-yref)**2)
            # obs_dist = cs.sqrt((x_obs[k]-xref)**2+(y_obs[k]-yref)**2)
            # c += cs.fmax(0.0, -(obs_dist-ego_dist)) * (cs.fmax(0.0, cone))
    cost += Qtx*(x-xtraj[-1])**2 + Qty*(y-ytraj[-1])**2 + Qttheta*(theta-thetaref)**2
    #cost += Qtx*(x-xref)**2 + Qty*(y-yref)**2 + Qttheta*(theta-thetaref)**2
    # -------Acceleration constraint
    (dv_c,dw_c) = (0,0)
    for t in range(0, nu*N, nu):
        uk = u[t:t+2]
        dv_c += cs.fmax(0.0, uk[0]-ukm1[0]-0.2)
        dw_c += cs.vertcat(cs.fmax(0.0, uk[1]-ukm1[1]-0.05),
                           cs.fmax(0.0, ukm1[1]-uk[1]-0.05))
        ukm1 = uk
    # -------Boundery constrint
    umin = []
    umax = []
    for i in range(N):
        umin.extend([vmin, wmin])
        umax.extend([vmax, wmax])

    set_c = og.constraints.Zero()
    set_y = og.constraints.BallInf(None, 1e12)
    bounds = og.constraints.Rectangle(umin, umax)
    C = cs.vertcat(dv_c, dw_c)
    problem = og.builder.Problem(u, p, cost).with_penalty_constraints(
        C).with_constraints(bounds).with_aug_lagrangian_constraints(c,set_c,set_y)

    build_config = og.config.BuildConfiguration()\
        .with_build_directory("python_test_build")\
        .with_build_mode("debug")\
        .with_tcp_interface_config()

    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("model_dd_opt")

    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(1e-5)\
        .with_initial_tolerance(1e-5)\
        .with_max_outer_iterations(7)\
        .with_max_duration_micros(200000)\
        .with_delta_tolerance(1e-4)\
        .with_penalty_weight_update_factor(5).with_initial_penalty(10).with_sufficient_decrease_coefficient(0.7)

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                              meta,
                                              build_config,
                                              solver_config) \
        .with_verbosity_level(1)

    builder.build()


if __name__ == '__main__':
    build_opt()
    import run_opt
