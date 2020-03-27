function [cTOT] = nonl_con_ipopt(Z,N,xk,uk,dt,obstacles,obstacles_u,r_obs) %[cin; ceq]
% ceq=zeros(N,1);
uk = uk(:);
xk = xk(:);
for i=1:N
    [A,B] = Linearized_discrete_DD_model(xk,uk,dt);    
    xk1=Z(N*3+3*i-2:N*3+3*i)';
    xk1=xk1(:);
    ceq(3*i-2:3*i,1)=xk1-A*xk-B*uk;
    xk=xk1;
    uk=Z(N*8+2*i-1:N*8+2*i);
    uk = uk(:);
end

iter = 1;
cin=zeros(length(obstacles)*N,1);

for j=1:length(obstacles)
    [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{j},obstacles_u{j},dt);
    for i = 1:N                
        obstacles{j}=A_obstacles*obstacles{j}+B_obstacles*obstacles_u{j};
        r=[Z(3*N+1+(3*(i-1)));Z(3*N+2+(3*(i-1)))]-obstacles{j}(1:2);
        vab=[Z(8*N+1+(2*(i-1)));Z(8*N+2+(2*(i-1)))]-[obstacles_u{j}(1);obstacles_u{j}(2)];
        
        d_square=-norm(r)^2+(dot(r,vab))^2/norm(vab)^2;
        cin(iter,1) = d_square+(r_obs)^2; % OBS OBS OBS; skall vara ^2
        iter = iter +1 ;
    end
end   

if (all(cin) == 0) % && isempty(obstacles)
    cin = zeros(2*N,1);
end
cTOT = [cin(:);ceq(:)];
