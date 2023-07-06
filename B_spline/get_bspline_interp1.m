function [ctrl_pts, u_vec] = get_bspline_interp1(time, way_pts, p, initCond, endCond)
%GET_BSPLINE_INTERP1 get parameters of the B-spline curve interplating through way points. (consider even/odd case of p)                                                    
%   Inputs:
%       time: 1 X n, time sequence at way points
%       way_pts: n X dim, waypoints
%       p: degree of b-spline
%       initCond: p/2 X dim(even p) or (p-1)/2 X dim(odd p), extra initial condition points (vel, acc, jerk, ...)
%       endCond: p/2 X dim(even p) or (p-1)/2 X dim(odd p), extra end condition points (vel, acc, jerk, ...)
%   Outputs:
%       ctrl_pts: m X dim, control points
%       u_vec: 1 X (n+2p+1) or 1 X (n+2p+2), knot vector

%% Get knot vector and number of control points
[N, dim] = size(way_pts);
if mod(p, 2) == 0 %even case
    u_vec = [ones(1,p+1)*time(1), (time(1:end-1)+time(2:end))/2, ones(1,p+1)*time(end)];
    m = N + p -1;
    numCond = p; %number of boundary conditions
else %odd case
    u_vec = [ones(1,p+1)*time(1), time(2:end-1), ones(1,p+1)*time(end)];
    m = N + p -2;
    numCond = p-1; %number of boundary conditions
end

%% Init and solve the linear equation
A = zeros(m+1, m+1);

c = zeros(m+1, dim);
c(1:N, dim) = way_pts;
c(N+1:end, dim) = [initCond; endCond];

id = 0 : m;

% waypoints
for i=1 : m+1-numCond
        A(i, :) = my_bsplineBasisDeriv(id, 0, p, u_vec, time(i));
end
% boundary conditions
k=0;
for i=m+1-numCond+1 : m+1-numCond/2
        k=k+1;
        A(i, :) = my_bsplineBasisDeriv(id, k, p, u_vec, time(1));
        A(i + numCond/2, :) = my_bsplineBasisDeriv(id, k, p, u_vec, time(end));
end

% solve
ctrl_pts = A\c;
end