function pts = bezierSurfaceEval(p, q, control_pts, u, v)
%BEZIERSURFACEEVAL bezier surface evaluation at specified (u, v) point sets using tensor product
%   inputs:
%       p, q: degree in u-direction, v-direction    
%       control_pts: n+1 X m+1 X dim tensor (u, v, dim)
%       u: 1 X N or N X 1 
%       v: 1 X M or M X 1
%   outputs:
%       pts: N x M x dim tensor

N = length(u);
M = length(v);
[n_plus1, m_plus1, ~] = size(control_pts);

%basis tensors
B_u_i = zeros(N, n_plus1);
B_v_j = zeros(M, m_plus1);

for i=1:n_plus1
    B_u_i(:, i) = bezierBasis(i-1, p, u);
end

for j=1:m_plus1
    B_v_j(:, j) = bezierBasis(j-1, q, v);
end

%compute tensor product
pts = tensorprod(B_v_j, control_pts, 2); %inner product at 2-nd index
pts = tensorprod(B_u_i, pts, 2); %inner product at 2-nd index