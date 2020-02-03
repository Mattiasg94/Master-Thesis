clc
clear all
% 
% y=1:10;
% x=ones(10,1)*3;
% o=3;
% xf=10;
% for i=1:10
%     
% obs(i)=1/((y(i)-o)+(x(i)-o))^2;
% obs2(i)=1/((y(i)-o)^2+(x(i)-o)^2);
% g(i)=((y(i)-xf)+(x(i)-xf))^2;
% goal(i)=(y(i)-xf)^2 + (x(i)-xf)^2;
% end
% obs(i)
% g(i)
% goal(i)
% subplot(3,1,1)
% plot(obs)
% subplot(3,1,2)
% plot(obs2)
% subplot(3,1,3)
% plot(goal)
lb=[1 0 -0.5 -0.5 ];
LB1=[];LB2=[];
for i=1:2
    LB1=[LB1,lb(1:2)];
    LB2=[LB2,lb(3:4)];
end
LB=[LB1,LB2]