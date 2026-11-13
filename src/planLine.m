function Q = planLine(ik, eeName, qStart, T0, T1, n)
%PLANLINE Straight-line Cartesian motion of the tool from T0 to T1.
%   The orientation is kept constant. The position follows a cosine profile
%   so that the motion starts and ends with zero velocity. Every sample is
%   solved with inverse kinematics, warm-started from the previous one.

    Q = zeros(n, numel(qStart));
    Q(1, :) = qStart;
    q = qStart;

    for k = 2:n
        s = (1 - cos(pi * (k - 1) / (n - 1))) / 2;   % 0 -> 1, smooth start and stop
        T = T0;
        T(1:3, 4) = (1 - s) * T0(1:3, 4) + s * T1(1:3, 4);
        % No random restarts here: a restart could jump to a different
        % arm configuration in the middle of the line
        q = solveIK(ik, eeName, T, q, "straight-line motion", false);
        Q(k, :) = q;
    end
end
