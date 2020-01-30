function f = AnonymousFunc(x,obstacle,N,xf)
f = 0;
for i = 1:2:2*N % OBS!! i = 1 bör ge 2 st f
    f = f + (x(i)-xf(1))^2 + (x(i+1)-xf(2))^2   ;
end

end

