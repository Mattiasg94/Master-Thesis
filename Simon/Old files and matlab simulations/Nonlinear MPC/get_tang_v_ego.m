function v_tang = get_tang_v_ego(v,x,y,th,center)
vx= -v*cos(th);
vy= -v*sin(th);
%     vx= -v*cs.cos(th)
%     vy= -v*cs.sin(th)

b_hat_x = (x-center(1)) / sqrt((x-center(1))^2+(y-center(2))^2);
b_hat_y = (y-center(2)) / sqrt((x-center(1))^2+(y-center(2))^2);
%     b_hat_x=(x-center[0])/cs.sqrt((x-center[0])**2+(y-center[1])**2)
%     b_hat_y=(y-center[1])/cs.sqrt((x-center[0])**2+(y-center[1])**2)

vb = vx*b_hat_x + vy*b_hat_y;
v_tang = v-sqrt(vb^2);
%     vb=vx*b_hat_x+vy*b_hat_y
%     v_tan=(v-cs.sqrt(vb**2))
end
