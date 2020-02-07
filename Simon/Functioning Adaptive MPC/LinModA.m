function [A] = LinModA(xr,ur,dt)

A = [ 1, 0, -dt*ur(1)*sin(xr(3));
      0, 1,  dt*ur(1)*cos(xr(3));
      0, 0,    1];
end