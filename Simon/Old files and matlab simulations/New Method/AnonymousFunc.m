function f = AnonymousFunc(x,obstacle,N,xf)
f = 0;
for i = 1:3:2*N % OBS!! i = 1 bör ge 2 st f
    f = f + (x(i)-xf(1))^2 + (x(i+1)-xf(2))^2; % + 1/(abs((x(i)-obstacle(1)))^2 + (x(i+1)-obstacle(2))^2);
end

end

