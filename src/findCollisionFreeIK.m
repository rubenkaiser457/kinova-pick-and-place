function q = findCollisionFreeIK(ik, eeName, T, qRef, robotCheck, obstacles, label)
%FINDCOLLISIONFREEIK IK solution that is reachable AND collision-free.
%   A 7-DOF arm can reach the same tool pose with many arm configurations
%   (elbow up/down/sideways). A plain IK solution may therefore reach the
%   pose with the elbow inside an obstacle. This function tries several
%   initial guesses, keeps the solutions without world collision and returns
%   the one closest to the reference configuration qRef.
%
%   robotCheck  robot model used for the collision check (e.g. including a
%               carried object)

    robot = ik.RigidBodyTree;

    % Initial guesses: reference rotated about the base axis towards the
    % target, the reference itself, then random configurations
    guesses = [rotateBaseTowards(robot, qRef, eeName, T); qRef];
    for k = 1:30
        guesses(end + 1, :) = randomGuess(robot, numel(qRef)); %#ok<AGROW>
    end

    best = [];
    bestDist = inf;
    nValid = 0;
    for k = 1:size(guesses, 1)
        try
            qTry = solveIK(ik, eeName, T, guesses(k, :), label, false);
        catch
            continue;   % pose not reached from this guess
        end

        isColliding = checkCollision(robotCheck, qTry, obstacles, "IgnoreSelfCollision", "on");
        if isColliding(end)
            continue;
        end

        nValid = nValid + 1;
        dist = norm(wrapAngle(qTry - qRef));
        if dist < bestDist
            best = qTry;
            bestDist = dist;
        end
        if nValid >= 5, break; end   % enough candidates to choose from
    end

    if isempty(best)
        error("pickplace:unreachable", ...
              "No collision-free arm configuration found for pose '%s'.", label);
    end
    q = best;
end

function q = rotateBaseTowards(robot, q, eeName, T)
% Rotate joint 1 (base) by the azimuth difference between the current tool
% position and the target position.
    p = tform2trvec(getTransform(robot, q, eeName));
    dAz = atan2(T(2, 4), T(1, 4)) - atan2(p(2), p(1));
    q(1) = wrapAngle(q(1) + dAz);
end

function q = randomGuess(robot, n)
    q = zeros(1, n);
    k = 0;
    for i = 1:robot.NumBodies
        jnt = robot.Bodies{i}.Joint;
        if jnt.Type == "fixed", continue; end
        k = k + 1;
        lim = max(min(jnt.PositionLimits, pi), -pi);
        q(k) = lim(1) + rand * (lim(2) - lim(1));
    end
end

function a = wrapAngle(a)
% Wrap angles to [-pi, pi)
    a = mod(a + pi, 2*pi) - pi;
end
