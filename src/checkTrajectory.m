function report = checkTrajectory(traj, scene)
%CHECKTRAJECTORY Verify the planned trajectory sample by sample.
%   Checks every sample against the real (not enlarged) table and obstacle and
%   reports the minimum clearance to the obstacle and the largest joint step.
%   While the object is lowered onto or lifted off the table it touches the
%   table by design, so these samples are only checked against the obstacle.

    n = size(traj.Q, 1);
    collidingSamples = [];
    minObstacleClearance = inf;

    for i = 1:n
        if traj.carrying(i)
            robot = traj.robotCarry;
        else
            robot = traj.robot;
        end

        if ismember(traj.phase(i), [3 5])
            obstacles = {scene.obstacle};
        else
            obstacles = scene.collisionObjects;
        end

        [isColliding, sepDist] = checkCollision(robot, traj.Q(i, :), obstacles, ...
            "IgnoreSelfCollision", "on", "Exhaustive", "on");
        if isColliding(end)
            collidingSamples(end + 1) = i; %#ok<AGROW>
        end

        % Last column of the separation matrix = distance to the obstacle
        d = sepDist(:, end);
        d = d(isfinite(d));
        if ~isempty(d)
            minObstacleClearance = min(minObstacleClearance, min(d));
        end
    end

    jointSteps = abs(diff(traj.Q));
    report.numSamples = n;
    report.collidingSamples = collidingSamples;
    report.minObstacleClearance = minObstacleClearance;
    report.maxJointStepDeg = rad2deg(max(jointSteps(:)));

    fprintf('\nTrajectory check\n');
    fprintf('  Samples:                    %d\n', n);
    fprintf('  Collision-free:             %s\n', string(isempty(collidingSamples)));
    fprintf('  Min. clearance to obstacle: %.1f cm\n', 100 * minObstacleClearance);
    fprintf('  Max. joint step per sample: %.2f deg\n\n', report.maxJointStepDeg);

    if ~isempty(collidingSamples)
        warning("pickplace:collision", "Collision detected in %d samples (first: %d).", ...
                numel(collidingSamples), collidingSamples(1));
        reportCollisions(traj, scene, collidingSamples);
    end
end

function reportCollisions(traj, scene, idx)
% Print which phase, which robot body and which obstacle are involved.
    fprintf('Colliding samples per phase:\n');
    for p = unique(traj.phase(idx))'
        inPhase = idx(traj.phase(idx) == p);
        fprintf('  Phase %d (%s): %d samples, %d..%d\n', p, traj.phaseNames(p), ...
                numel(inPhase), inPhase(1), inPhase(end));
    end

    i = idx(1);
    if traj.carrying(i), robot = traj.robotCarry; else, robot = traj.robot; end
    obstacles = scene.collisionObjects;
    names = ["table", "obstacle"];
    [~, sepDist] = checkCollision(robot, traj.Q(i, :), obstacles, ...
        "IgnoreSelfCollision", "on", "Exhaustive", "on");
    worldCols = sepDist(:, end - numel(obstacles) + 1:end);
    bodyNames = [string(robot.BodyNames), string(robot.BaseName)];
    fprintf('Sample %d (phase %d):\n', i, traj.phase(i));
    for b = 1:size(worldCols, 1)
        for o = 1:numel(obstacles)
            d = worldCols(b, o);
            if isnan(d) || d < 0.005
                if b <= numel(bodyNames), bn = bodyNames(b); else, bn = "body " + b; end
                fprintf('  %s touches %s\n', bn, names(o));
            end
        end
    end
    fprintf('  joints [deg]: %s\n', mat2str(round(rad2deg(traj.Q(i, :)), 1)));
end
