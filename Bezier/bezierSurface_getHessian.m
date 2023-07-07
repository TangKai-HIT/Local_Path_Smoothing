function hessian = bezierSurface_getHessian(ctrl_pts, u, v)
%BEZIERSURFACE_GETHESSIAN get hessian on bezier surface
%   inputs:
%       control_pts: m+1 X n+1 X dim tensor (u, v, dim)
%       u: 1 X N or N X 1 
%       v: 1 X M or M X 1
%   Outputs:
%       hessian: 2 X 2 X dim, hessian of surface at (u,v) 

%% init
[M, N, dim] = size(ctrl_pts);
m = M-1;  n = N-1;

hessian = zeros(2, 2, dim);

%% Get isoparametric control points
%Get v-parametric curve control points on u
B_u_j = zeros(1, M);

for i=1:M
    B_u_j(i) = bezierBasis(i-1, m, u);
end

ctrl_pts_v = tensorprod(ctrl_pts, B_u_j, 1, 2); 

%Get u-parametric curve control points on v
B_v_j = zeros(1, N);

for i=1:N
    B_v_j(i) = bezierBasis(i-1, n, v);
end

ctrl_pts_u = tensorprod(ctrl_pts, B_v_j, 2, 2); 

%% Compute diagonal terms
%ddp_ddu
[ctrl_pts_ddu, ddn] = bezierDerivParams(m, 2, ctrl_pts_u);
hessian(1,1,:) = bezierEval(ddn, u, ctrl_pts_ddu);

%ddp_ddv
[ctrl_pts_ddv, ddm] = bezierDerivParams(n, 2, ctrl_pts_v);
hessian(2,2,:) = bezierEval(ddm, v, ctrl_pts_ddv);

%% Compute off-diagonal terms
%ddp_dudv & dp_dvdu
dB_v_j = zeros(1, N-1);
for i=1:N-1
    dB_v_j(i) = bezierBasis(i-1, n-1, v);
end

ctrl_pts_u_dv = n * tensorprod(diff(ctrl_pts, 1, 2), dB_v_j, 2, 2); %diff along v & tensor product
ctrl_pts_du_dv = m * diff(ctrl_pts_u_dv, 1, 1);

hessian(1,2,:) = bezierEval(m-1, u, ctrl_pts_du_dv);
hessian(2,1,:) = hessian(1,2,:);