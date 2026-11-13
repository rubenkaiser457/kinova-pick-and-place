function Q = planJointMotion(planner, robot, qStart, qGoal)
%PLANJOINTMOTION Collision-free joint-space motion with RRT.
%   1. RRT finds a collision-free path (sparse waypoints).
%   2. The path is shortened to remove unnecessary detours.
%   3. Each straight joint-space segment is sampled with a trapezoidal
%      velocity profile, so the robot accelerates and decelerates smoothly.
%   Segments between the shortened waypoints are validated by the planner,
%   so the sampled trajectory stays collision-free.

    [path, info] = plan(planner, qStart, qGoal);
    if ~info.IsPathFound
        error("pickplace:planning", "RRT could not find a collision-free path.");
    end
    path = shorten(planner, path, 200);

    % Continuous joints (unlimited rotation): the planner connects e.g. 179 deg
    % and -179 deg the short way (2 deg). Unwrapping makes the interpolation
    % below follow the same short way instead of turning by 358 deg.
    cont = continuousJoints(robot);
    path(:, cont) = unwrap(path(:, cont));

    samplesPerRad = 25;
    Q = path(1, :);
    for i = 1:size(path, 1) - 1
        a = path(i, :);
        b = path(i + 1, :);
        n = max(15, ceil(max(abs(b - a)) * samplesPerRad));
        q = trapveltraj([a' b'], n);
        Q = [Q; q(:, 2:end)']; %#ok<AGROW>
    end
end

function cont = continuousJoints(robot)
% Logical index of joints whose range covers a full turn.
    cont = false(1, 0);
    for i = 1:robot.NumBodies
        jnt = robot.Bodies{i}.Joint;
        if jnt.Type == "fixed", continue; end
        cont(end + 1) = diff(jnt.PositionLimits) >= 2*pi; %#ok<AGROW>
    end
end
