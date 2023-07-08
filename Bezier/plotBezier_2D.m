function plotBezier_2D(ax, control_pts)
%PLOTBSPLINE_2D plot 2D Bezier

%Sample
u=linspace(0, 1, 100);
m = size(control_pts, 1) - 1;
pts = bezierEval(m, u, control_pts);

%plot control polygon
hold on;
plot(ax, control_pts(:, 1), control_pts(:, 2), '--o', 'Color', 'blue', LineWidth=0.8);

%plot curve
plot(ax, pts(:, 1), pts(:, 2), '-r', LineWidth=1);