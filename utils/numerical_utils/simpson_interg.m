function [f_x, x] = simpson_interg(funcHandle, interval, m)
%SIMPSON_INTERG composite simpson intergration
%   Inputs:
%       funcHandle: function handle -- df(x)
%       interval: integration interval ([a, b])
%       m: number of divided panels (2h = (b - a)/m)
%   Outputs:
%       x: query points -- x0, x1, ... , xm
%       f_x: intergration points -- f(x0), ... ,f(xm) [note that f(x0)=0 for integrate from x0 to x0]

x = linspace(interval(1), interval(2), m+1); %query points
f_x = zeros(1, m+1); %intergration points
h = (interval(2) - interval(1)) / (2*m);

%init
df_temp = zeros(1, m+1); %function evaluation at x0~xm
f_x = zeros(1, m+1); %intergration results at x0~xm

%intergration
df_temp(1) = funcHandle(x(1)); %f(a)

for i=2:m+1
    df_temp(i) = funcHandle(x(i));
    df_mid = funcHandle(x(i) - h);
    f_x(i) = f_x(i-1) + h/3*(df_temp(i-1) + 4*df_mid + df_temp(i));
end

end

