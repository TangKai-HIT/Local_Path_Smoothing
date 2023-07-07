function [e_u, e_v, n] = bezierSurface_getTanNorm(ctrl_pts, u, v)
%BEZIERSURFACE_GETTANNORM get normalised tangent and normal vector on bezier surface
%   Outputs:
%       e_u: 1 X dim, tangent in u-direction
%       e_v: 1 X dim, tangent in v-direction
%       n: 1 X dim, normal vector of surface at (u,v) 

[N, M, ~] = size(ctrl_pts);
p = N-1;  q = M-1;

%Get v-parametric curve control points on u
B_u_j = zeros(1, N);

for i=1:N
    B_u_j(i) = bezierBasis(i-1, p, u);
end

ctrl_pts_v = tensorprod(ctrl_pts, B_u_j, 1, 2); 

%Get u-parametric curve control points on v
B_v_j = zeros(1, M);

for i=1:M
    B_v_j(i) = bezierBasis(i-1, q, v);
end

ctrl_pts_u = tensorprod(ctrl_pts, B_v_j, 2, 2); 

%Compute u - tangent vector 
e_u = bezierDerivOnce(p, u, ctrl_pts_u);
e_u = e_u / norm(e_u);

%Compute v - tangent vector 
e_v = bezierDerivOnce(q, v, ctrl_pts_v);
e_v = e_v / norm(e_v);

%Compute normal vector
n = cross(e_u, e_v);
n = n / norm(n);