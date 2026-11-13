function scene = createScene()
%CREATESCENE Table, cylindrical obstacle and the object to pick.
%   All dimensions in metres, robot base at the origin. The robot is
%   mounted at the height of the table surface (tableTop = 0), which keeps
%   all grasp poses well inside the workspace of the Kinova Gen3.

    scene.tableTop      = 0.00;          % height of the table surface
    scene.tableCenterXY = [0.55 0.00];
    scene.tableSize     = [0.60 1.20];   % [x y]
    tableThickness      = 0.05;

    scene.table = collisionBox(scene.tableSize(1), scene.tableSize(2), tableThickness);
    scene.table.Pose = trvec2tform([scene.tableCenterXY, scene.tableTop - tableThickness/2]);

    scene.floor = collisionBox(2.0, 2.0, 0.02);
    scene.floor.Pose = trvec2tform([0.3 0 scene.tableTop - tableThickness - 0.01]);   % visual only

    % Obstacle between the object and the default target position
    scene.obstacleXY     = [0.50 0.00];
    scene.obstacleRadius = 0.08;
    obstacleHeight       = 0.35;
    scene.obstacle = collisionCylinder(scene.obstacleRadius, obstacleHeight);
    scene.obstacle.Pose = trvec2tform([scene.obstacleXY, scene.tableTop + obstacleHeight/2]);

    % Object to pick, standing on the table
    scene.objectSize = [0.05 0.05 0.10];
    scene.object = collisionBox(scene.objectSize(1), scene.objectSize(2), scene.objectSize(3));
    scene.object.Pose = trvec2tform([0.45 -0.30 scene.tableTop + scene.objectSize(3)/2]);

    % Real collision objects (used to verify the trajectory). The floor is
    % excluded because the robot base stands on it.
    scene.collisionObjects = {scene.table, scene.obstacle};

    % The planner works with an obstacle enlarged by a safety margin, so the
    % planned motion keeps a distance instead of only just touching it
    scene.safetyMargin = 0.03;
    obstaclePlanning = collisionCylinder(scene.obstacleRadius + scene.safetyMargin, ...
                                         obstacleHeight + 2*scene.safetyMargin);
    obstaclePlanning.Pose = scene.obstacle.Pose;
    scene.planningObstacles = {scene.table, obstaclePlanning};
end
