function b = bezierDerivOnce(m, u, control_pts)
%BEZIERDERIVONCE evaluate first-order derivative of m-degree bezier curve at u using Casteljau algorithm
%   Inputs:
%       control_pts: m X dim
%       u: 1 X N or N X 1

new_control_pts = diff(control_pts, 1, 1);

b = m * bezierEval(m-1, u, new_control_pts);