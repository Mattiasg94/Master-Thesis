function [B] = LinModB(xr,ur,dt)

B = [ dt*cos(xr(3)) 0;
      dt*sin(xr(3)) 0;
      0     dt];
  
end