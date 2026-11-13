%% Kinova Gen3 pick-and-place with collision-free motion planning
% Plans and animates a complete pick-and-place cycle:
%   ready pose -> above object -> grasp -> lift -> above target -> place -> retreat -> ready pose
%
% Free-space motions are planned with RRT (manipulatorRRT), shortened and
% time-parameterized with trapezoidal velocity profiles. Approach and retreat
% motions are straight vertical lines solved with inverse kinematics.
%
% Requirements: MATLAB with Robotics System Toolbox

clear; clear functions; clc; close all;   %#ok<CLFUNC> reload edited functions
rehash;

root = fileparts(mfilename('fullpath'));
if isempty(root), root = pwd; end
addpath(fullfile(root, 'src'));

%% Configuration
cfg.targetXY    = [0.45 0.30];  % place position on the table [x y] in m
cfg.rngSeed     = 7;            % RRT is randomized: fixed seed for reproducible runs
cfg.exportMedia = true;         % write media/pick_and_place.gif and .mp4
cfg.mediaFolder = fullfile(root, 'media');

rng(cfg.rngSeed);

%% Robot and scene
[robot, eeName, gripperHasVisual] = buildKinovaWithGripper();
scene = createScene();

%% Motion planning
fprintf('Planning pick-and-place to target [%.2f %.2f] ...\n', cfg.targetXY);
tic;
traj = planPickAndPlace(robot, eeName, scene, cfg.targetXY);
fprintf('Planning finished in %.1f s, %d trajectory samples.\n', toc, size(traj.Q, 1));

%% Verification
checkTrajectory(traj, scene);

%% Animation
animatePickAndPlace(robot, scene, traj, gripperHasVisual, cfg.exportMedia, cfg.mediaFolder);
