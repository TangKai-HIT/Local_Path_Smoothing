%demo3: cubic bezier spiral G2 transition test
%Note: cubic bezier spiral (CBS)
close all; clear; clc;

%% Test segments
arc_start = [0;0];
arc_startHeading = pi;
R = -6;
radius = abs(R);
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
phi_e = deg2rad(20); %P6 position angle on arc

%CBS params
m = 3; %order
C1 = 0.58;

K = radius * tan(phi_e/2);
lambda = C1;

h = (lambda + 4) * K / (6 * cos(phi_e));
g = lambda * h;

%% Get control points
[B3, tau3, ~] = arc.getPointFrenet(phi_e);

%get B2
B2 = B3 - K * tau3;

%get B1
tau1 = arc.startTau;
B1 = B2 - h * tau1;

%get B0
B0 = B1 - g * tau1;

%% Plot result
controlPts = [B0, B1, B2, B3]';
plotBezier_2D(ax, controlPts);
grid on;

%% Get kappa on CBS
num = 50;
u_sample = linspace(0, 1, num);
%derivative of CBS
[dp_control_pts, ~] = bezierDerivParams(m, 1, controlPts);
[ddp_control_pts, ~] = bezierDerivParams(m-1, 1, dp_control_pts);
%compute kappa & arc length
kappa = zeros(num, 1);
smooth_len = 0;
for i=1:num
    cur_P = bezierEval(m, u_sample(i), controlPts);
    if i>1
        smooth_len = smooth_len + norm(cur_P - last_P);
    end
    last_P = cur_P;

    dp_du = bezierEval(m-1, u_sample(i), dp_control_pts);
    ddp_ddu = bezierEval(m-2, u_sample(i), ddp_control_pts);
    kappa(i) = abs(get_kappa(dp_du', ddp_ddu'));
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
grid on;