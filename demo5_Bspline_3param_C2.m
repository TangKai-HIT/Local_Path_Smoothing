%demo4: cubic b-spline local C2 smoothing method1(with 3 design params) on a tangent arc-line pair 
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
c = sqrt(6)/6; %line smoothing ratio
% c = 0.2;
l_e = 0.2; %line smoothing length
phi_e = deg2rad(10); %smoothing angle on arc
%b-spline params
p = 3; %order
u_vec = [zeros(1,p+1), 0.5, ones(1,p+1)]; %5 control points, m=4

%% Get control points on line
P2 = arc.startPt;

P0 = P2 - l_e * line.tau;
P1 = P0 + c * l_e * line.tau;

%% Get control points on arc
%control points on arc
radius = abs(R);
[P4, tau4, ~] = arc.getPointFrenet(phi_e);
P3 = P4 - sqrt(6) / 3 * radius * sin(phi_e/2) * tau4;

%% Plot result
controlPts = [P0, P1, P2, P3, P4]';
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