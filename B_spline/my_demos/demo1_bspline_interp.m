%demo1 -- B-spline interpolation with p=4, 5
clc; clear; close all;
addpath(genpath('../'));

%% Params
time = [0, 5, 7, 8, 10, 15, 18];
waypoints = [3; -2; -5; 0; 6; 12; 8];
initCond = [2; 0];
endCond = [-3; 0];

%% Get B-spline for interpolation
% with p = 4
[ctrl_pts4, u_vec4] = get_bspline_interp1(time, waypoints, 4, initCond, endCond);
% with p = 5
[ctrl_pts5, u_vec5] = get_bspline_interp1(time, waypoints, 5, initCond, endCond);

%% Plot B-spline
