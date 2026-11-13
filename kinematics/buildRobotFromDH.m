function robot = buildRobotFromDH(dh, T_tool)
%BUILDROBOTFROMDH Rigid body tree from a DH table (revolute joints).
%   Each link gets a simple box so the model is visible in plots.

    robot = rigidBodyTree("DataFormat", "row");
    parent = robot.BaseName;

    for i = 1:size(dh, 1)
        body = rigidBody("link" + i);
        joint = rigidBodyJoint("joint" + i, "revolute");
        setFixedTransform(joint, dh(i, :), "dh");
        body.Joint = joint;

        % Box between the joint frames, sized by the link parameters
        len = max(0.05, hypot(dh(i, 1), dh(i, 3)));
        box = collisionBox(0.05, 0.05, len);
        box.Pose = trvec2tform([0 0 -len/2]);
        addCollision(body, box);

        addBody(robot, body, parent);
        parent = body.Name;
    end

    tool = rigidBody("tool");
    toolJoint = rigidBodyJoint("toolMount", "fixed");
    setFixedTransform(toolJoint, T_tool);
    tool.Joint = toolJoint;
    addBody(robot, tool, parent);
end
