import opengen as og
import casadi.casadi as cs
import numpy as np

# Build parametric optimizer
# ------------------------------------
ts = 0.1
(nu, nx, N) = (2, 3, 60)
(xref, yref, thetaref) = (2, 2, 0)
(Qx, Qy, Qtheta) = (10, 10, 1)
(Rv, Rw) = (1, 1)
(Qtx, Qty, Qttheta) = (100, 100, 1)

def build_opt():

    u = cs.SX.sym('u', nu*N)
    z0 = cs.SX.sym('z0', nx)
    (x, y, theta) = (z0[0], z0[1], z0[2])
    cost = 0
    c = 0

    for t in range(0, nu*N, nu):
        u_t = u[t:t+2]
        cost += Qx*(x-xref)**2 + Qy*(y-yref)**2 + Qtheta*(theta-thetaref)**2
        cost += Rv*u_t[0]**2+Rw*u_t[1]**2
        x += ts*cs.cos(theta)*u_t[0]
        y += ts*cs.sin(theta)*u_t[0]
        theta += ts*u_t[1]
        c += cs.fmax(0, 0.5 - x**2 - y**2)

    cost += Qtx*(x-xref)**2 + Qty*(y-yref)**2 + Qttheta*(theta-thetaref)**2

    umin = [-1.0] * (nu*N)
    umax = [1.0] * (nu*N)
    bounds = og.constraints.Rectangle(umin, umax)
    set_c = og.constraints.Zero()
    set_y = og.constraints.BallInf(None, 1e12)

    problem = og.builder.Problem(u, z0, cost).with_penalty_constraints(c).with_constraints(bounds)

    build_config = og.config.BuildConfiguration()\
        .with_build_directory("python_test_build")\
        .with_build_mode("debug")\
        .with_tcp_interface_config()

    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("model_dd_opt")

    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(1e-4)\
        .with_initial_tolerance(1e-4)\
        .with_max_outer_iterations(5)\
        .with_delta_tolerance(1e-2)\
        .with_penalty_weight_update_factor(10.0).with_initial_penalty(100)

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                              meta,
                                              build_config,
                                              solver_config) \
        .with_verbosity_level(1)

    builder.build()


if __name__ == '__main__':
    build_opt()
    import test2
