function b = bezierEval(m, u, control_pts)
%BEZIEREVAL evaluate bezier curve at u using Casteljau algorithm
%   Inputs:
%       control_pts: m-1 X dim
%       u: 1 X N or N X 1

b = zeros(length(u), size(control_pts, 2));
%% Iteration
for n = 1:length(u)
    iter_ctrl_pts = control_pts;
    for k=1:m
        for j = 1:m-k+1
            iter_ctrl_pts(j, :) = (1-u(n)) * iter_ctrl_pts(j, :) + u(n) * iter_ctrl_pts(j+1, :);
        end
    end
    b(n, :) =  iter_ctrl_pts(1, :);
end

