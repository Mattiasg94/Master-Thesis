%%
clc
clear all
close all
R1 = [1 1];
R2 = [2.5 2.5];
radius = 0.5;
xlim([0 4]);
ylim([0 4]);

circle(R2(1),R2(2), radius);
R1R2 = R2-R1;
l = sqrt(norm(R1R2)^2 - radius^2);
gamma = atan2(R1R2(2),R1R2(1));
alpha = atan2(radius,l);
beta = gamma - alpha;
lambda12 = [cos(beta)*l+R1(1);
            sin(beta)*l+R1(2)]; 
hold on
plot([R1(1);lambda12(1)],[R1(2);lambda12(2)]);
hold on
plot([R1(1); R2(1)],[R1(1); R2(2)]);

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
% hold on 
% scatter(X,Y,'g')


function h = circle(x,y,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
% hold off
end

%%
% clc
% clear all
% close all










