function [cin,ceq] = nonlcon(Z,N,uk,obstacles)

cin=0;
ceq=[];

min_dist=1; % low=close to

if any(obstacles)~=0
    j = 1;
    for i = 3*N+1:3:6*N     % (Num states+ num inputs) *N
        
        obs_dist_const(j:j+1) = [-sqrt((Z(i)-obstacles(1))^2 + (Z(i+1)-obstacles(2))^2) + min_dist;
                                 -sqrt((Z(i)-obstacles(3))^2 + (Z(i+1)-obstacles(4))^2) + min_dist];
        j = j+2;
    end
    
    obs_dist_const = obs_dist_const(:);
    cin = obs_dist_const;
end

end

