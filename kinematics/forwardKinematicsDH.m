function T = forwardKinematicsDH(dh, q, T_tool)
%FORWARDKINEMATICSDH Forward kinematics from a DH table.
%   dh     n-by-4 table [a alpha d theta_offset], one row per revolute joint
%   q      1-by-n joint angles in rad
%   T_tool fixed transform from the last joint frame to the tool
%
%   T = A1(q1) * A2(q2) * ... * An(qn) * T_tool

    T = eye(4);
    for i = 1:size(dh, 1)
        T = T * dhTransform(dh(i, 1), dh(i, 2), dh(i, 3), dh(i, 4) + q(i));
    end
    T = T * T_tool;
end
