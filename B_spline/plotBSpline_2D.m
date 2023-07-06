function plotBSpline_2D(ax, control_pts, p, u_vec)
%PLOTBSPLINE_2D plot 2D B-Spline

%Sample
u=linspace(u_vec(1), u_vec(end), 500);
pts = my_bsplineEval(control_pts, p, u_vec, u);

%plot control polygon
hold on;
plot(ax, control_pts(:, 1), control_pts(:, 2), '--o', 'Color', 'blue', LineWidth=0.6);

%plot curve
plot(ax, pts(:, 1), pts(:, 2), '-r', LineWidth=0.8);