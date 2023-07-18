%demo4-1: cubic bezier deformed spiral G2 transition test of line-air pair
%with gap
%Note: cubic bezier spiral (CBS)
% close all; clear; clc;
close all;

%% Test segments
gap = 0.2; %gap >= 0.018*R/2
arc_start = [0;gap];
arc_startHeading = pi;

R = -6;
radius = abs(R);
arc_kappa = abs(1/R);

theta = deg2rad(60);
arc = CircleSegment2D(arc_start, arc_startHeading, R, theta);

line_start = [5; 0];
line_end = [0; 0];
line = LineSegment(line_start, line_end);

arcSamples = arc.makeSamples(50);
figure()
hold on;
ax = gca;
plot(arcSamples(1,:), arcSamples(2,:),  '-k', 'LineWidth', 0.7);
plot([line.startPt(1), line.endPt(1)], [line.startPt(2), line.endPt(2)],  '-k', 'LineWidth', 0.7);
axis equal; grid on;

%% Set basic parameters
%design parameters
phi_e = deg2rad(15); %P6 position angle on arc

%lambda computation switching threshold of gap
gap_threshold = 0.018*radius/2*(tan(deg2rad(10)/2) + 1);

%CBS params
m = 3; %order
C1 = 0.58;

K = radius * tan(phi_e/2) + gap/sin(phi_e);

if gap < gap_threshold 
    disp("use deformed CBS...");
    load("./results/fitted_lookup.mat");
    lambda = fitted_lookup(rad2deg(phi_e));
else
    disp("use general CBS...");
    a = 2*gap/(radius*(1 + tan(phi_e/2)^2)) + 1; %fix coefficient
%     a =  1;
    phi_max = 2*atan(sqrt((2/0.018)*gap/radius-1));

    phi_e = min(phi_e, phi_max);
    fprintf("phi: %.2f\n", rad2deg(phi_e));

    lambda = a * 9/2*(1 - tan(phi_e/2)^2) - 4;
end

% h = (lambda + 4) * K / (6 * cos(phi_e));
h = arc_kappa * (3*K^2) / (2*sin(phi_e)); %no longer satisfy monototic kappa, but get continuous kappa instead 
g = lambda * h;

disp("line occupation length:")
disp(g+h+cos(phi_e)*K - radius*sin(phi_e)); %test if length on line decrease as phi_e decrease
% %Test Result: length on line decreases with phi_e when phi_e <= 20 degree

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

%% Uniform arc-length sampling on bezier curve
% %get param to arc length index
% derivativeFcn = @(u) norm(bezierEval(m-1, u, dp_control_pts));
% [arc_lens, params] = arclength_reparam(derivativeFcn, [0, 1], 100);
% 
% %plot s-u relation
% figure()
% plot(arc_lens, params, '-b', 'LineWidth', 1);
% xlabel("s"); ylabel("u");
% 
% %resample on s
% num_samples = 20;
% [s_sample, u_sample] = sample_on_s(params, arc_lens, num_samples, true);
% 
% %plot resample points on curve
% figure()
% hold on; grid on;
% delta_s_sample = [];
% pre_pt = [];
% for i = 1:num_samples
%     cur_pt = bezierEval(m, u_sample(i), controlPts);
%     if i > 1
%         delta_s_sample = [delta_s_sample, norm(pre_pt - cur_pt)];
%     end
% 
%     plot(cur_pt(1), cur_pt(2), 'ok', 'MarkerSize', 3, 'MarkerFaceColor','black');
%     pre_pt = cur_pt;
% end
% plotBezier_2D(gca, controlPts); 
% axis equal;