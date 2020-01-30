clc
clear all


A=[1 0;1 0];
B=[1 0;0 1];
C=[1 0;0 1];
sys=ss(A,B,C,0);
Ts=1;
sysd=c2d(sys,Ts);

N=1;

Nsim=40;
a=zeros(Nsim , 1);
u = zeros(Nsim , 2);
x = zeros(Nsim + 1, 2);
x0=[0;0];
x(1,:) = x0;
xf=10;
uL=0.1;
uU=1;

for k = 2:Nsim+1
    Z = optimizer_fmincon(A, B, N, xf, x(k-1,:)',uL,uU);
    %Z = optimizer_lin(A, B, N, xf, x(k-1,:)',uL,uU);
    %Z = optimizer(A, B, N, xf, x(k-1,:)',uL,uU);
    if isempty(Z)
       break
    end
    
    u(k-1,:)=Z(N*2+1:N*2+2);
    x(k,:) = A*x(k-1,:)' + B*u(k-1,:)';
    t(k-1)=Z(length(Z)-(2*N-1));
    a(k-1)=Z(length(Z)-(N-1));
    subplot(2,1,1);
    plot(x(:,1),x(:,2),'*')
    title("x,y")
    subplot(2,1,2);
    plot(u(:,1),u(:,2),'*')
    title("u1,u2")
    pause(0.1)
end
disp("done")


    
