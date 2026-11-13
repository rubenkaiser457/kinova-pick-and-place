%% Verify a hand-written DH forward kinematics against the Robotics System Toolbox
% A 6-DOF arm with PUMA 560 link dimensions is described by its DH table.
% The forward kinematics is computed in two independent ways:
%   1. own implementation: product of the DH link transforms (forwardKinematicsDH)
%   2. Robotics System Toolbox: rigidBodyTree built from the same DH table
% Both must give the same tool pose for random joint configurations.

clear; clc; close all;
here = fileparts(mfilename('fullpath'));
if ~isempty(here), addpath(here); end

% DH table [a alpha d theta_offset] (PUMA 560 dimensions in m)
dh = [ 0       pi/2   0        0;
       0.4318  0      0        0;
       0.0203 -pi/2   0.15005  0;
       0       pi/2   0.4318   0;
       0      -pi/2   0        0;
       0       0      0        0 ];
T_tool = trvec2tform([0 0 0.1]);   % tool 10 cm along the last joint axis

robot = buildRobotFromDH(dh, T_tool);

rng(1);
N = 1000;
maxPosError = 0;
maxRotError = 0;
for k = 1:N
    q = randomConfiguration(robot);
    T_own     = forwardKinematicsDH(dh, q, T_tool);
    T_toolbox = getTransform(robot, q, "tool");

    maxPosError = max(maxPosError, norm(T_own(1:3, 4) - T_toolbox(1:3, 4)));
    R = T_own(1:3, 1:3)' * T_toolbox(1:3, 1:3);
    maxRotError = max(maxRotError, acos(max(-1, min(1, (trace(R) - 1) / 2))));
end

fprintf('Compared %d random configurations\n', N);
fprintf('  max. position error:    %.2e m\n', maxPosError);
fprintf('  max. orientation error: %.2e rad\n', maxRotError);

% Show one configuration with the tool frame from the own implementation
q = randomConfiguration(robot);
T = forwardKinematicsDH(dh, q, T_tool);
figure('Color', 'w');
show(robot, q, 'Collisions', 'on', 'Visuals', 'off', 'Frames', 'off');
hold on;
plotTransforms(tform2trvec(T), tform2quat(T), 'FrameSize', 0.15);
title('DH model: tool frame from own forward kinematics');
axis equal; view(135, 25);
