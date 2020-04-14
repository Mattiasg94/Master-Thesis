clc
clear
close all 
bra = [3.9061e+000     1.2065e+000   184.5858e-003
     4.8891e+000     1.3901e+000   165.8780e-003
     5.8754e+000     1.5552e+000   186.6427e-003
     6.8580e+000     1.7407e+000   225.7264e-003
     7.8327e+000     1.9645e+000   533.9350e-003
     8.6935e+000     2.4735e+000   835.8355e-003
     9.3640e+000     3.2153e+000   637.7361e-003
    10.1675e+000     3.8107e+000   -60.3633e-003
    11.1657e+000     3.7504e+000    -1.2585e+000
    11.3193e+000     3.2746e+000    -1.9566e+000];

kass =   [   4.5894e+000     1.3866e+000   124.8495e-003
     5.5816e+000     1.5112e+000   131.2800e-003
     6.5731e+000     1.6421e+000   114.1682e-003
     7.5666e+000     1.7560e+000    91.1609e-003
     8.5625e+000     1.8471e+000    55.0150e-003
     9.5612e+000     1.9020e+000  -173.1177e-003
    10.5462e+000     1.7298e+000    39.7579e-003
    11.0684e+000     1.7506e+000   752.6334e-003
    11.0849e+000     1.7660e+000     1.9655e+000
    10.8839e+000     2.2484e+000     2.7285e+000];
kass2 =[4.5894    1.3866    1.3832    0.0033841
    5.5816    1.5112     1.485      0.02619
    6.5731    1.6421    1.5761      0.06598
    7.5666     1.756    1.6607     0.095321
    8.5625    1.8471    1.7396      0.10746
    9.5612     1.902    1.8108     0.091239
    10.546    1.7298    1.8686     -0.13883
    11.068    1.7506    1.9046     -0.15404
    11.085     1.766     1.907     -0.14101
    10.884    2.2484    1.8608      0.38763];

kass3 = [
    12.9830e+000     1.8562e+000   -73.9195e-003
    13.4936e+000     1.7999e+000   414.6214e-003
    14.4988e+000     2.1661e+000   530.6137e-003
    15.4709e+000     2.6972e+000   160.3327e-003
    16.4586e+000     2.8569e+000  -248.5094e-003
    17.5175e+000     2.6138e+000  -170.2493e-003
    18.5774e+000     2.4490e+000   264.5150e-003
    19.5547e+000     2.7124e+000   206.4440e-003
    20.6023e+000     2.9163e+000   639.9544e-003
    22.0045e+000     3.3272e+000     1.5733e+000
];
kass4 = [11.8659e+000     2.6742e+000   399.6473e-003
    11.8659e+000     2.6742e+000   900.8665e-003
    11.8659e+000     2.6742e+000     1.6111e+000
    11.8659e+000     2.6742e+000     1.8214e+000
    11.8659e+000     2.6742e+000     1.5317e+000
    11.8716e+000     2.8619e+000   744.0539e-003
    11.8716e+000     2.8632e+000  -177.3128e-003
    12.3645e+000     2.7752e+000  -599.7653e-003
    13.0689e+000     2.2918e+000  -528.4988e-003
    13.6150e+000     1.9713e+000  -350.1338e-003];

x = kass(:,1);
y = kass(:,2);
grade = 4;
figure(1)
plot(x,y,'o')
title('Plot of y Versus t')
[p,S] = polyfit(x,y,grade);
% [~,~,mu] = polyfit(x,y,grade);
% [p,S,mu] = polyfit(x,y,3);
% mu
t2 = min(x):(max(x)-min(x))/(length(x)-1):max(x);
[y2,delta] = polyval(p,t2,S);
T = table(x(:),y(:),y2(:),y-y2(:),'VariableNames',{'X','Y','Fit','FitError'});
figure(2)
plot(x,y,'o',t2,y2)
title('Plot of Data (Points) and Model (Line)')
immse(y,y2(:))
% var(y-y2(:))



%%

clc;
clear all;
close all;

p1 = [1; 3];
p2 = [5; 4];
P = p2-p1;
plot([p1(1) p2(1)], [p1(2); p2(2)])
xlim([0 8]);
ylim([0 8]);
dt = 0.2;
v = norm(P)/dt;







