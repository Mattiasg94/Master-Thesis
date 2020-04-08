N = 10;
Nsim=100;
dt=0.2;
ur=[0.2;0];
xr=[20;4;0];

ylimit = [7; 1];
lanes = [6;4;2];
follow_traj = false;
Y_REF = [11, 10.99, 10.96, 10.91, 10.84, 10.76, 10.65, 10.53, 10.39, 10.24, 10.08, 9.91, 9.72, 9.53, 9.34, 9.14, 8.94, 8.74, 8.55, 8.35, 8.17, 7.99, 7.82, 7.67, 7.53, 7.4, 7.29, 7.19, 7.12, 7.06, 7.02, 7.0, 7.0, 7.03, 7.07, 7.13, 7.21, 7.3, 7.42, 7.55, 7.69, 7.85, 8.02, 8.2, 8.39, 8.58, 8.78, 8.98, 9.17, 9.37, 9.57, 9.76, 9.94, 10.11, 10.27, 10.42, 10.55, 10.67, 10.77, 10.85, 10.92, 10.97, 10.99];
X_REF = [0.0, 0.16, 0.32, 0.48, 0.65, 0.81, 0.97, 1.13, 1.29, 1.45, 1.61, 1.77, 1.94, 2.1, 2.26, 2.42, 2.58, 2.74, 2.9, 3.06, 3.23, 3.39, 3.55, 3.71, 3.87, 4.03, 4.19, 4.35, 4.52, 4.68, 4.84, 5.0, 5.16, 5.32, 5.48, 5.65, 5.81, 5.97, 6.13, 6.29, 6.45, 6.61, 6.77, 6.94, 7.1, 7.26, 7.42, 7.58, 7.74, 7.9, 8.06, 8.23, 8.39, 8.55, 8.71, 8.87, 9.03, 9.19, 9.35, 9.52, 9.68, 9.84, 10.0];
THETA_REF=[0.0, -0.07841, -0.156035, -0.2321, -0.3058475, -0.37654, -0.44347, -0.5059675, -0.56341, -0.6152225, -0.66089, -0.6999525, -0.7320225, -0.7567775, -0.77397, -0.78343, -0.7850625, -0.7788525, -0.7648575, -0.7432225, -0.71416, -0.6779625, -0.6349925, -0.585675, -0.5305075, -0.47004, -0.404875, -0.3356625, -0.2631, -0.187905, -0.110835, -0.0326575, 0.0458475, 0.1238925, 0.2007025, 0.275505, 0.347555, 0.4161325, 0.4805525, 0.54017, 0.5943925, 0.6426725, 0.684535, 0.719555, 0.7473875, 0.76775, 0.7804425, 0.7853375, 0.782385, 0.7716175, 0.7531375, 0.7271325, 0.6938625, 0.6536625, 0.6069275, 0.55413, 0.495795, 0.4325075, 0.3648975, 0.2936425, 0.2194525, 0.14307, 0.0652575];

if follow_traj
    x0=[X_REF(1);Y_REF(1);THETA_REF(1)];
else
    x0=[0;6;0];
end
reference_lane = xr(2); % (x0(2) == lanes(1))*x0(2) +(x0(2) == lanes(2))*x0(2) +(x0(2) == lanes(3))*x0(2);
lane_st.lanes = lanes;
lane_st.reference_lane = reference_lane;
lane_st.ylimit = ylimit;
u0=[0.5;0];
lb_x=[-inf ylimit(2)+0.2 -2*pi];
ub_x=[inf ylimit(1)-0.2 2*pi];
lb_u=[0 -3];
ub_u=[2 3];
dv=1;
dw=1;
lb=[-inf -inf -inf lb_x -inf -inf lb_u];
ub=[ inf  inf  inf ub_x  inf  inf ub_u];
time_it = 0;
R_jerk = 3;

%% plot
obstacles = {[15;6;pi], [10;4;pi]}; %, [10;2;0]}; %,   ,[15;4;0]};
obstacles_u = {[-0.5;0], [-0.5;0]}; %, [-0.5;0]}; % ,   ,[0.1;0]};
plot_obstacles = plot(1); textbox=plot(1); plot_obstacles_radius=plot(1); plot_obstacles_safety_radius = plot(1);
r_obs=1;
r_safety_margin = 0.2;

% scatter(xr(1),xr(2),50,'y','LineWidth',5);
if follow_traj
    viscircles([X_REF(end),Y_REF(end)],0.1,'Color','y','Linewidth',5);
else
    viscircles([xr(1),xr(2)],0.1,'Color','y','Linewidth',5);
end
xlim([0,22])
ylim([-1,10])

xlim([0,22])
ylim([-1,10])
hold on
plot([0 22],[ylimit(1) ylimit(1)],'b','linewidth',1.5)
hold on
plot([0 22],[ylimit(end) ylimit(end)],'b','linewidth',1.5)
hold on

for i = 1:length(lane_st.lanes)-1
    plot([0 22],[(ylimit(1)-ylimit(2))*i/3+ylimit(2) (ylimit(1)-ylimit(2))*i/3+ylimit(2)],'b','linewidth',1)
    hold on
end

Vfval=zeros(Nsim,1);
Vfval(1)=inf;
feasible=1;
close_obstacles=obstacles;
close_obstacles_u=obstacles_u;
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta,MR_jerk]=setup(x0,u0,xr,ur,ub,lb,Nsim,N,R_jerk);