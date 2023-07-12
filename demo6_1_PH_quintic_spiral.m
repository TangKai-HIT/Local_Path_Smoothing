%demo6-1: a generalised PH quintic spiral G2 transition test
close all; clear; clc;
% close all;

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
phi = deg2rad(10); %P5 position angle on arc
phi_e = phi/2; 
m = 0.6714; % single curvature extremum condition: m > 0.6714

%PH curve params
p = 5; %order

%% Get PH curve params & control points
%PH curve params
d1 = sqrt(18 + 5*m + 3*cos(2*phi_e));
d0 = (2*sqrt(6) * (d1 - sqrt(6)*cos(phi_e)) * sin(phi_e)) / (3 + m);
d = sqrt(d0 * radius);

u1 = d^3 / (4 * radius * sin(phi_e));
u0 = m*u1;
u2 = d*cos(phi_e);
u_coeff = [u0, u1, u2];

v_coeff = [0, 0, d*sin(phi_e)];

%control points
controlPts = get_PH_controlPts(u_coeff, v_coeff);

%transformation
trans2D = @(theta, trans_vec, point) (rot2d(theta)*point + trans_vec);
P5 = arc.getPointFrenet(phi);
P0_heading = arc.startHeading;

for i =1:size(controlPts, 1)
    controlPts(i, 2) = -controlPts(i, 2); %flip
    controlPts(i, :) = trans2D(P0_heading, arc.startPt, controlPts(i, :)');
end

controlPts = controlPts + (P5' - controlPts(end, :));

%% Plot result
plotBezier_2D(ax, controlPts);
grid on;

%% Get kappa on CBS
num = 50;
u_sample = linspace(0, 1, num);
%derivative of CBS
[dp_control_pts, ~] = bezierDerivParams(p, 1, controlPts);
[ddp_control_pts, ~] = bezierDerivParams(p-1, 1, dp_control_pts);
%compute kappa & arc length
kappa = zeros(num, 1);
smooth_len = 0;
for i=1:num
    cur_P = bezierEval(p, u_sample(i), controlPts);
    if i>1
        smooth_len = smooth_len + norm(cur_P - last_P);
    end
    last_P = cur_P;

    kappa(i) = abs(get_PH_curvature(u_sample(i), u_coeff, v_coeff));
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