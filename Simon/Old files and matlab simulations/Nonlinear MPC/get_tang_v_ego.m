function v_tang = get_tang_v_ego(v,x,y,th,center)

vx= v*cos(th);
vy= v*sin(th);

b_hat_x = (x-center(1)) / sqrt((x-center(1))^2+(y-center(2))^2);
b_hat_y = (y-center(2)) / sqrt((x-center(1))^2+(y-center(2))^2);

vb = vx*b_hat_x + vy*b_hat_y;
v_tang = v-abs(vb);
end
