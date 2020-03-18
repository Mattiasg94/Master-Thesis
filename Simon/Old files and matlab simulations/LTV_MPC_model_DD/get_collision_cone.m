function [lambda_r,lambda_f,Vec_lambda_r,Vec_lambda_f] = get_collision_cone(R1,R2,radius)

R1R2 = R2-R1;
l = sqrt(norm(R1R2)^2 - radius^2);
gamma = atan2(R1R2(2),R1R2(1));
alpha = atan2(radius,l);
beta = gamma - alpha;
lambda_f = [cos(beta)*l+R1(1);
            sin(beta)*l+R1(2)]; 
beta2 = gamma + alpha;
lambda_r = [cos(beta2)*l+R1(1);
            sin(beta2)*l+R1(2)]; 
Vec_lambda_f = [R1(1) R1(2);  lambda_f(1) lambda_f(2)];      
Vec_lambda_r = [R1(1) R1(2);  lambda_r(1) lambda_r(2)];      

