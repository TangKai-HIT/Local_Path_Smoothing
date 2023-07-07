function gradient = bezierSurface_getGrad(ctrl_pts, u, v)
%BEZIERSURFACE_GETGRAD get gradient on bezier surface
%   inputs:
%       control_pts: n+1 X m+1 X dim tensor (u, v, dim)
%       u: 1 X N or N X 1 
%       v: 1 X M or M X 1
%   Outputs:
%       gradient: 2 X dim, gradient of surface at (u,v) 

[N, M, dim] = size(ctrl_pts);
p = N-1;  q = M-1;

gradient = zeros(2, dim);

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
gradient(1, :) = bezierDerivOnce(p, u, ctrl_pts_u);

%Compute v - tangent vector 
gradient(2, :) = bezierDerivOnce(q, v, ctrl_pts_v);