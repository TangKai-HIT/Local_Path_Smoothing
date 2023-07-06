function [e_u, e_v, n] = bsplineSurface_getTanNorm(ctrl_pts, p, q, u_vec, v_vec, u, v)
%BSPLINESURFACE_GETTANNORM get normalised tangent and normal vector on b-spline surface
%   Outputs:
%       e_u: 1 X dim, tangent in u-direction
%       e_v: 1 X dim, tangent in v-direction
%       n: 1 X dim, normal vector of surface at (u,v) 

[N, M, ~] = size(ctrl_pts);

%Get v-parametric curve control points on u
B_u_j = zeros(1, N);

for i=1:N
    B_u_j(i) = my_bsplineBasis(i-1, p, u_vec, u);
end

ctrl_pts_v = tensorprod(ctrl_pts, B_u_j, 2); 

%Get u-parametric curve control points on v
B_v_j = zeros(1, M);

for i=1:M
    B_v_j(i) = my_bsplineBasis(i-1, q, v_vec, v);
end

ctrl_pts_u = tensorprod(ctrl_pts, B_v_j, 2); 

%Compute u - tangent vector 
[d_ctrl_pts_u, d_u_vec] = my_bsplineDerivOnce(ctrl_pts_u, u_vec, p);
e_u = my_bsplineEval(d_ctrl_pts_u, p-1, d_u_vec, u);
e_u = e_u / norm(e_u);

%Compute v - tangent vector 
[d_ctrl_pts_v, d_v_vec] = my_bsplineDerivOnce(ctrl_pts_v, v_vec, q);
e_v = my_bsplineEval(d_ctrl_pts_v, q-1, d_v_vec, v);
e_v = e_v / norm(e_v);

%Compute normal vector
n = cross(e_u, e_v);