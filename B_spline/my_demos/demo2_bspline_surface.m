%demo2: generate a b-sline controlled surface
addpath(genpath('../../'));
%% init params 
p = 2; q=3;
u_vec = [0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1];
v_vec = [0, 0, 0, 0, 0.33, 0.66, 1, 1, 1, 1];

control_pts = zeros(6,6,3);
rng(2);
for i = 1:6
    for j=1:6
        control_pts(i,j,:) = [i, j, randn(1)];
    end
end

%% show surface
plotBSplineSurface(gca, p, q, u_vec, v_vec, control_pts, 0:0.02:1, 0:0.02:1);