function q = solveIK(ik, eeName, T, qGuess, label, allowRestarts)
%SOLVEIK Inverse kinematics with reachability check and random restarts.
%   The numerical IK solver can get stuck in a local minimum depending on
%   the initial guess. If the first attempt from qGuess does not reach the
%   pose within 2 mm and 2 degrees, the solver is restarted from random
%   configurations (unless allowRestarts is false). An error is raised if
%   no attempt reaches the pose.

    if nargin < 6, allowRestarts = true; end

    robot = ik.RigidBodyTree;
    ik.SolverParameters.MaxIterations = 1500;
    ik.SolverParameters.MaxTime = 2;
    weights = [1 1 1 1 1 1];

    nRestarts = 0;
    if allowRestarts, nRestarts = 20; end

    bestErr = inf;
    for attempt = 0:nRestarts
        if attempt == 0
            guess = qGuess;
        else
            guess = randomGuess(robot, numel(qGuess));
        end

        qTry = ik(eeName, T, weights, guess);
        [posErr, rotErr] = poseError(robot, qTry, eeName, T);

        if posErr <= 2e-3 && rotErr <= deg2rad(2)
            q = qTry;
            return;
        end
        if posErr + rotErr < bestErr
            bestErr = posErr + rotErr;
            bestPos = posErr;
            bestRot = rotErr;
        end
    end

    error("pickplace:unreachable", ...
          "Pose '%s' is outside the workspace (best position error %.1f mm, orientation error %.1f deg).", ...
          label, 1e3 * bestPos, rad2deg(bestRot));
end

function q = randomGuess(robot, n)
% Random configuration within the joint limits. Continuous joints have
% infinite limits, so they are sampled in [-pi, pi].
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

function [posErr, rotErr] = poseError(robot, q, eeName, T)
    T_reached = getTransform(robot, q, eeName);
    posErr = norm(T_reached(1:3, 4) - T(1:3, 4));
    R = T_reached(1:3, 1:3)' * T(1:3, 1:3);
    rotErr = acos(max(-1, min(1, (trace(R) - 1) / 2)));
end
