%demo2: quintic b-spline local C3 smoothing method1(with 2 control params) on a tangent arc-line pair 
close all; clear; clc;

%% Test segments
arc_start = [0;0];
arc_startHeading = pi;
R = -6;
arc_kappa = abs(1/R);
theta = deg2rad(60);
arc = CircleSegment2D(arc_start, arc_startHeading, R, theta);

line_start = [5; 0];
line_end = arc_start;
line = LineSegment(line_start, line_end);

arcSamples = arc.makeSamples(50);
figure()
hold on;
ax = gca;
plot(arcSamples(1,:), arcSamples(2,:),  '-k', 'LineWidth', 0.7);
plot([line.startPt(1), line.endPt(1)], [line.startPt(2), line.endPt(2)],  '-k', 'LineWidth', 0.7);
axis equal

%% Set basic parameters
%design parameters
l = 0.2;
c1_o = 0.3; %P6-P5:length ratio
phi_e = deg2rad(5); %P6 position angle on arc
% phi_e = theta/2;
%b-spline params
p = 5; %order
u_vec = [zeros(1,6), 0.5, ones(1,6)]; %7 control points, m=6
%junction
dis_P3 = 0;
P3 = line.getPointFromEnd(dis_P3);

%control points on line
P0 = line.getPointFromEnd(2.5 * l + dis_P3);
P1 = line.getPointFromEnd(2 * l + dis_P3);
P2 = line.getPointFromEnd(l + dis_P3);

%% Get control points on arc
l_o = 2*sin(phi_e/2); %normalized

%get alpha
tan_alpha = 2.5*(c1_o^3 * l_o) / (1/20 + c1_o^2);
alpha = atan(tan_alpha);

%get c2_o
c2_o = (1/(20 * c1_o ) + c1_o)  / cos(alpha);

%control points on arc
radius = abs(R);
[P6, tau6, ~] = arc.getPointFrenet(phi_e);
P5 = P6 - radius * c1_o * l_o * tau6;
P4 = P5 - radius * c2_o * l_o * rot2d(alpha) * tau6;

%% Plot result
controlPts = [P0, P1, P2, P3, P4, P5, P6]';
plotBSpline_2D(ax, controlPts, p, u_vec);

%% Get kappa on b-spline
num = 20;
u_sample = linspace(0, 1, num);
%derivative b-spline
[dp_control_pts, dp_u_vec] = my_bsplineDerivOnce(controlPts, u_vec, p);
[ddp_control_pts, ddp_u_vec] = my_bsplineDerivOnce(dp_control_pts, dp_u_vec, p-1);
%compute kappa & arc length
kappa = zeros(num, 1);
smooth_len = 0;
for i=1:num
    cur_P = my_bsplineEval(controlPts, p, u_vec, u_sample(i));
    if i>1
        smooth_len = smooth_len + norm(cur_P - last_P);
    end
    last_P = cur_P;

    dp_du = my_bsplineEval(dp_control_pts, p-1, dp_u_vec, u_sample(i));
    ddp_ddu = my_bsplineEval(ddp_control_pts, p-2, ddp_u_vec, u_sample(i));
    dtau_ds = get_dtau_ds(dp_du, ddp_ddu);
    kappa(i) = norm(dtau_ds);
end
%print smoothing length
disp("smoothing length:"); disp(smooth_len);
%print max kappa
max_kappa = max(kappa);
disp("max kappa:"); disp(max(kappa));
if max_kappa > arc_kappa
    overshoot = (max_kappa - arc_kappa) / arc_kappa;
else
    overshoot = 0;
end
disp("Kappa Overshoot:"); fprintf("%% %.2f\n", overshoot*100);

%plot kappa varying
figure();
plot(u_sample, kappa, '-b', "LineWidth", 1.5); hold on;
plot(u_sample, ones(1, num) * (1/radius), '--k', "LineWidth", 1.5);
plot(u_sample, ones(1, num) * max(kappa), '--r', "LineWidth", 1.5);