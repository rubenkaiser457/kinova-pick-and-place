function [robot, eeName, gripperHasVisual] = buildKinovaWithGripper()
%BUILDKINOVAWITHGRIPPER Kinova Gen3 (7 DOF) with a simple fixed gripper.
%   The gripper is modelled as a rigid box mounted 8 cm in front of the
%   tool flange. Its frame is used as the tool center point (TCP).

    robot = loadrobot("kinovaGen3", "DataFormat", "row", "Gravity", [0 0 -9.81]);
    flange = robot.BodyNames{end};

    gripper = rigidBody("gripper");
    mount = rigidBodyJoint("gripperMount", "fixed");
    setFixedTransform(mount, trvec2tform([0 0 0.08]));
    gripper.Joint = mount;

    gripperSize = [0.04 0.04 0.06];
    addCollision(gripper, collisionBox(gripperSize(1), gripperSize(2), gripperSize(3)));

    % Visual geometry is only supported in newer releases. Without it the
    % gripper is drawn through its collision geometry.
    gripperHasVisual = true;
    try
        addVisual(gripper, "Box", gripperSize);
    catch
        gripperHasVisual = false;
    end

    addBody(robot, gripper, flange);
    eeName = gripper.Name;
end
