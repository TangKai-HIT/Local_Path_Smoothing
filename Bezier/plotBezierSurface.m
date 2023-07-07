function plotBezierSurface(ax, p, q, control_pts, u_range, v_range)
%PLOTBEZIERSURFACE plot 3D Bezier surface patch
%   input:
%       u_range, v_range: sampled points on param range

hold(ax,"on");
% u_samp = linspace(u_range(1), u_range(2), numSamples);
% v_samp = linspace(v_range(1), v_range(2), numSamples);
pts = bezierSurfaceEval(p, q, control_pts, u_range, v_range);

surf(ax, pts(:, :, 1), pts(:, :, 2), pts(:, :, 3));
axis equal;
