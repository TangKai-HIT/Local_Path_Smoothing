function [new_control_pts, new_u_vec] = my_bsplineDerivOnce(control_pts, u_vec, p)
%MY_BSPLINEDERIVONCE get derivative of the p-degree spline once, return equivalent control points and knot vector
%   Inputs:
%       control_pts: m X dim
%       u_vec
%       p
%   Outputs:
%       new_control_pts: m X dim, equivalent control points
%       new_u_vec: new knots vector

new_u_vec = u_vec(2 : end-1);
[m, dim] = size(control_pts);
new_control_pts = zeros(m-1, dim);

for j = 1:m-1
    denom = u_vec(j+p+1) - u_vec(j+1);
    if denom == 0
        new_control_pts(j, :) = zeros(1, dim);
    else
        new_control_pts(j, :) = p*(control_pts(j+1,:) - control_pts(j,:))/denom;
    end
end