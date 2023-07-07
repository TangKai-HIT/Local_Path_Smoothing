function [newCtrlPts, newDegree] = bezierDerivParams(m, diffOrder, control_pts)
%BEZIERDERIVPARAMS return new degree & control points of differentiated m-degree bezier curve at u
%   Inputs:
%       m: degree
%       diffOrder: order of differentiation
%       control_pts: m+1 X dim

newDegree = m - diffOrder;
newCtrlPts = control_pts;

for i=1:diffOrder
    newCtrlPts = (m - i +1) * diff(newCtrlPts, 1, 1);
end