function traj = planPickAndPlace(robot, eeName, scene, targetXY)
%PLANPICKANDPLACE Plan the complete pick-and-place cycle.
%   Returns a struct with the joint trajectory (one row per sample), the
%   phase of every sample, the object pose per sample and the robot models
%   used for planning.
%
%   Phases:
%     1  ready pose -> above object       (RRT, joint space)
%     2  descend to grasp                 (straight line, IK)
%     3  lift object                      (straight line, IK)
%     4  transport to above target        (RRT with carried object)
%     5  descend and place                (straight line, IK)
%     6  retreat upwards                  (straight line, IK)
%     7  return to ready pose             (RRT, joint space)

    validateTarget(scene, targetXY);

    ik = inverseKinematics('RigidBodyTree', robot);
    R_down = axang2tform([0 1 0 pi]);               % tool z-axis points downwards
    poseAt = @(xy, z) trvec2tform([xy z]) * R_down;  % TCP pose above the table

    % Heights of the gripper frame (centre of the gripper body)
    gripperHalfHeight = 0.03;
    clearance = 0.005;
    graspZ = scene.tableTop + scene.objectSize(3) + gripperHalfHeight + clearance;
    hoverZ = scene.tableTop + 0.25;
    nLine  = 30;                                     % samples per straight-line motion

    objStart = tform2trvec(scene.object.Pose);
    objXY = objStart(1:2);

    %% Key configurations
    % Key poses are solved with a collision check, because a 7-DOF arm can
    % reach the same tool pose with different elbow positions
    obstacles = scene.planningObstacles;
    qReady    = findCollisionFreeIK(ik, eeName, poseAt([0.35 0], scene.tableTop + 0.35), ...
                                    homeConfiguration(robot), robot, obstacles, "ready pose");
    qPreGrasp = findCollisionFreeIK(ik, eeName, poseAt(objXY, hoverZ), qReady, robot, obstacles, "pre-grasp");

    %% Planners: robot alone and robot carrying the object
    plannerFree = makePlanner(robot, scene.planningObstacles);

    %% 1) ready -> above object
    Q1 = planJointMotion(plannerFree, robot, qReady, qPreGrasp);

    %% 2) descend to grasp
    Q2 = planLine(ik, eeName, qPreGrasp, poseAt(objXY, hoverZ), poseAt(objXY, graspZ), nLine);
    qGrasp = Q2(end, :);

    % Object pose relative to the gripper at the moment of grasping. It stays
    % constant while the object is carried.
    T_objInEE = getTransform(robot, qGrasp, eeName) \ scene.object.Pose;
    robotCarry = attachObject(robot, eeName, T_objInEE, scene.objectSize);
    plannerCarry = makePlanner(robotCarry, scene.planningObstacles);

    %% 3) lift
    Q3 = planLine(ik, eeName, qGrasp, poseAt(objXY, graspZ), poseAt(objXY, hoverZ), nLine);

    %% 4) transport to above target
    qPrePlace = findCollisionFreeIK(ik, eeName, poseAt(targetXY, hoverZ), Q3(end, :), ...
                                    robotCarry, obstacles, "pre-place");
    Q4 = planJointMotion(plannerCarry, robot, Q3(end, :), qPrePlace);

    %% 5) descend and place
    Q5 = planLine(ik, eeName, qPrePlace, poseAt(targetXY, hoverZ), poseAt(targetXY, graspZ), nLine);

    %% 6) retreat
    Q6 = planLine(ik, eeName, Q5(end, :), poseAt(targetXY, graspZ), poseAt(targetXY, hoverZ), nLine);

    %% 7) back to ready pose
    Q7 = planJointMotion(plannerFree, robot, Q6(end, :), qReady);

    %% Assemble trajectory (short pauses while grasping and releasing)
    holdFrames = 10;
    segments = {Q1, Q2, repmat(qGrasp, holdFrames, 1), Q3, Q4, Q5, ...
                repmat(Q5(end, :), holdFrames, 1), Q6, Q7};
    phases   = [1, 2, 2, 3, 4, 5, 5, 6, 7];

    traj.Q = [];
    traj.phase = [];
    for k = 1:numel(segments)
        seg = segments{k};
        if k > 1 && ~ismember(k, [3 7])     % drop duplicated start sample
            seg = seg(2:end, :);
        end
        traj.Q = [traj.Q; seg];
        traj.phase = [traj.phase; repmat(phases(k), size(seg, 1), 1)];
    end

    % Make continuous joints continuous across segment boundaries as well
    % (the same physical pose can be expressed with angles differing by 2*pi)
    traj.Q = unwrap(traj.Q);

    traj.carrying = ismember(traj.phase, [3 4 5]);
    traj.phaseNames = ["Move to object", "Grasp", "Lift", "Transport", ...
                       "Place", "Retreat", "Return to ready pose"];

    %% Object pose for every sample
    T_final = getTransform(robot, Q5(end, :), eeName) * T_objInEE;
    n = size(traj.Q, 1);
    traj.objectPose = repmat(scene.object.Pose, 1, 1, n);
    for i = 1:n
        if traj.carrying(i)
            traj.objectPose(:, :, i) = getTransform(robot, traj.Q(i, :), eeName) * T_objInEE;
        elseif traj.phase(i) >= 6
            traj.objectPose(:, :, i) = T_final;
        end
    end

    traj.robot = robot;
    traj.robotCarry = robotCarry;
    traj.eeName = eeName;
    traj.targetXY = targetXY;
end

%% ------------------------------------------------------------------------
function validateTarget(scene, xy)
    margin = 0.05;
    halfSize = scene.tableSize / 2 - margin;
    if any(abs(xy - scene.tableCenterXY) > halfSize)
        error("pickplace:target", "Target [%.2f %.2f] is not on the table.", xy);
    end
    if norm(xy - scene.obstacleXY) < scene.obstacleRadius + 0.06
        error("pickplace:target", "Target [%.2f %.2f] overlaps with the obstacle.", xy);
    end
    objPos = tform2trvec(scene.object.Pose);
    if norm(xy - objPos(1:2)) < 0.08
        error("pickplace:target", "Target is identical to the start position of the object.");
    end
end

function planner = makePlanner(robot, obstacles)
    planner = manipulatorRRT(robot, obstacles);
    planner.MaxConnectionDistance = 0.3;
    planner.ValidationDistance = 0.02;
    % The collision meshes of adjacent Kinova links overlap in the model, so
    % self-collision checks would reject almost every configuration.
    planner.IgnoreSelfCollision = true;
end

function robotCarry = attachObject(robot, eeName, T_objInEE, objSize)
    robotCarry = copy(robot);
    body = rigidBody("carriedObject");
    mount = rigidBodyJoint("carriedObjectMount", "fixed");
    setFixedTransform(mount, T_objInEE);
    body.Joint = mount;
    addCollision(body, collisionBox(objSize(1), objSize(2), objSize(3)));
    addBody(robotCarry, body, eeName);
end
