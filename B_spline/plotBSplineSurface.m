function plotBSplineSurface(ax, p, q, u_vec, v_vec, control_pts, u_range, v_range)
%PLOTBSPLINESURFACE plot 3D B-Spline surface patch
%   input:
%       u_range, v_range: sampled points on param range

hold(ax,"on");
% u_samp = linspace(u_range(1), u_range(2), numSamples);
% v_samp = linspace(v_range(1), v_range(2), numSamples);
% pts = bsplineSurfaceEval(p, q, u_vec, v_vec, control_pts, u_samp, v_samp);
pts = bsplineSurfaceEval(p, q, u_vec, v_vec, control_pts, u_range, v_range);

surf(ax, pts(:, :, 1), pts(:, :, 2), pts(:, :, 3));
axis equal;
