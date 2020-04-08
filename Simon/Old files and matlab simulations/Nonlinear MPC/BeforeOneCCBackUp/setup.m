function [u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta,MR_jerk]=setup(x0,u0,xr,ur,ub,lb,Nsim,N,R_jerk,Q,R)
u = zeros(Nsim , 2);
x = zeros(Nsim + 1, 3);
x(1,:) = x0;
u(1,:) = u0;
u_tilde = zeros(Nsim , 2);
x_tilde = zeros(Nsim + 1, 3);

lb1=[];lb2=[];lb3=[];lb4=[];
ub1=[];ub2=[];ub3=[];ub4=[];
for i=1:N
    lb1=[lb1,lb(1:3)];
    lb2=[lb2,lb(4:6)];
    lb3=[lb3,lb(7:8)];
    lb4=[lb4,lb(9:10)];

    ub1=[ub1,ub(1:3)];
    ub2=[ub2,ub(4:6)];
    ub3=[ub3,ub(7:8)];
    ub4=[ub4,ub(9:10)];
end

lb=[lb1,lb2,lb3,lb4];
ub=[ub1,ub2,ub3,ub4];
Z0=0;
MQ=[];
MR=[];
for i=1:N
   MQ=blkdiag(MQ,Q); 
   MR=blkdiag(MR,R);    
end
Mxr=[];
Mur=[];
for i=1:N
    Mxr=[Mxr;xr];
    Mur=[Mur;ur];
end
u1_delta=[1,0,-1,0];
u2_delta=[0,1,0,-1];
Mu1_delta=zeros(N-1,2*N);
Mu2_delta=zeros(N-1,2*N);
for i=1:N-1
   Mu1_delta(i,2*(i-1)+1:2*(i-1)+4)=u1_delta;
   Mu2_delta(i,2*(i-1)+1:2*(i-1)+4)=u2_delta;
end
MR_jerk = eye(N);
for i = 1:N
    for j = 1:N
        if i == j
            MR_jerk(i,j+1) = -1;
        end
    end
end
MR_jerk = R_jerk*MR_jerk(:,1:end-1);


