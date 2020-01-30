clc
N = 3;
% e = ones(N,1);
% A = spdiags([ones(N,1) ones(N,1)],0:1,N,N+1);
% A
% full(A)
A=zeros(N,N*2)
for i=1:N
    A(i,2*i-1:2*i)=ones(1,2)
end

