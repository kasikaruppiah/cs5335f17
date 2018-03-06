% Modified PRM on puma 560
function project()
    % create a puma 560 robot
    mdl_puma560;
    
    % initialize the obstacles 3 x 1 matrix representing the center of
    % spherical obstacle and a radius
    sphereCenter1 = [-0.5; 0.5; 0.0];
    sphereRadius1 = 0.1;
    sphereCenter2 = [1.0; -1.0; 0.0];
    sphereRadius2 = 0.2;
    sphereCenter3 = [0.5; 1.0; -1.0];
    sphereRadius3 = 0.3;
    sphereCenter4 = [-0.75; -0.75; -0.75];
    sphereRadius4 = 0.5;
    % group the obstacles centers as a 3 x n matrix
    sphereCenter = [sphereCenter1, sphereCenter2, sphereCenter3, sphereCenter4];
    % group the obstacles radius as a 1 x n matrix
    sphereRadius = [sphereRadius1, sphereRadius2, sphereRadius3, sphereRadius4];
    
    % initialize the start and the goal positions as 3 x 1 coordinate
    % matrix
    xStart = [-0.75; 0.0; 0.0];
    xGoal = [0.5; 0.5; 0.5];
    
    % Convert the 3-D coordinates into 4-D configuration space
    qStart = p560.ikine6s(transl(xStart));
    qGoal = p560.ikine6s(transl(xGoal));

    % plot the starting position of the robot and the obstacles
    p560.plot(qStart);
    hold on;
    for i = 1 : length(sphereRadius)
        drawSphere(sphereCenter(:, i), sphereRadius(i));
    end

    qMilestones = [];
    retryCount = 0;
    retries = 5;

    % retry to find the qMilestones for max of 5 times till the milestones are found 
    while retryCount < retries && isempty(qMilestones)
        qMilestones = milestones(p560, sphereCenter, sphereRadius, qStart, qGoal);
        retryCount = retryCount + 1;
    end
    
    if qMilestones
        % smooth the milestones to achieve a better path
        qMilestonesSmoothed = smoothMilestones(p560, qMilestones, sphereCenter, sphereRadius);
        qTrajectory = interpolateMilestones(qMilestonesSmoothed);
        % plot the milestones as trajectories
        p560.plot(qTrajectory);
    else
        % when no milestones are found after 5 retries
        fprintf('No valid path found even after %d retries\n', retries);
    end
end

% Draws a sphere on the plot with given radius at the specified location
% input: position -> 3x1 position of center of sphere
%        radius -> radius of sphere
function drawSphere(position, radius)
    [X, Y, Z] = sphere;

    X = X * radius + position(1);
    Y = Y * radius + position(2);
    Z = Z * radius + position(3);

    surf(X, Y, Z);
end

% Calculate a path from qStart to qGoal
% input: robot -> a puma 560 robot
%        sphereCenter -> 3xN position of center of spherical obstacle
%        sphereRadius -> 1xN radius of obstacle
%        qStart -> 1x4 joint vector describing starting configuration of arm
%        qGoal -> 1x4 joint vector describing starting configuration of arm
% output -> qMilestones -> 4xn vector of milestones. A straight-line interpolated
%               path through these milestones should result in a collision-free path.
%               You may output any number of milestones. The first milestone
%               should be qStart. The last milestone should place the end effector at xGoal.
function qMilestones = milestones(p560, sphereCenter, sphereRadius, qStart, qGoal)
    % number of free configurations
    N = 50;
    % distance between neighbouring configurations
    DIST = 1.0;

    n = 1;
    % add the start configuration
    qTree = qStart;
    adj = zeros(1);

    while n < N + 2
        if n < N + 1
            % generate a random configuration
            qRand = - pi + 2 * pi * rand(1, 6);
        else
            % add the goal configuration
            qRand = qGoal;
        end

        % check if the configuration is already in the roadmap
        if ismember(qRand, qTree, 'rows') == 0
            xRand = p560.fkine(qRand);
            xRand = xRand(1 : end - 1, end);

            % check if the configuration is not on a obstacle
            if checkPointIsFree(xRand, sphereCenter, sphereRadius) == 1
                qNear = [];
                [m, ~] = size(qTree);
                for i = 1 : m
                    qTemp = qTree(i, :);
                    xTemp = p560.fkine(qTemp);
                    xTemp = xTemp(1 : end - 1, end);

                    qTempDist = sqrt(sum((xRand - xTemp) .^ 2));

                    % find all connected components to the random
                    % configuration present in the roadmap
                    if qTempDist < DIST
                        if checkCollision(p560, qRand, qTemp, sphereCenter, sphereRadius) == 0
                            qNear(end + 1) = i;
                        end
                    end
                end

                qTree = [qTree; qRand];
                n = n + 1;

                m = length(adj);
                adj = [adj; zeros(1, m)];
                adj = [adj zeros(m + 1, 1)];

                % update the adjaceny matrix
                if qNear
                    for i = 1 : length(qNear)
                        adj(n, qNear(i)) = 1;
                        adj(qNear(i), n) = 1;
                    end
                end
            end
        end
    end

    % create a graph from the adjacency matrix
    G = graph(adj);
    % find the shortest path using djikistra's algorithm
    path = shortestpath(G, 1, n);
    % return the configurations in the path
    qMilestones = qTree(path, :);
end

% check if a coordinate is in the list of obstacles
function free = checkPointIsFree(xRand, sphereCenter, sphereRadius)
    for i = 1 : length(sphereRadius)
        free = checkPointInSphere(xRand, sphereCenter(:, i), sphereRadius(i));
        if free == 0
            break
        end
    end
end

% Check if a coordinate is inside a sphere
function free = checkPointInSphere(xRand, sphereCenter, sphereRadius)
    if sqrt(sum((xRand - sphereCenter) .^ 2)) > sphereRadius
        free = 1;
    else
        free = 0;
    end
end

% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% input: robot -> puma 560 robot
%        qStart, qEnd -> start and end configuration, respectively. Both are 1x4
%           vectors.
%        sphereCenter -> 3xN position of center of sphere
%        r -> 1xN radius of sphere
% output: collision -> binary number that denotes whether this
%           configuration is in collision or not.
function collision = checkCollision(robot, qStart, qEnd, sphereCenter, sphereRadius)
    points = 11;
    % delta q - increments for evenly spaced points between qStart and qEnd
    qDelta = (qEnd - qStart) / points;
    % Total 12 points
    % q1, 10 points between q1 and q2, q2
    for i = 0 : points
        qRand = qStart + i * qDelta;
        % check for each obstacle
        for j = 1 : length(sphereRadius)
            collision = robotCollision(robot, qRand, sphereCenter(:, j), sphereRadius(j));
            if collision == 1
                break
            end
        end
        if collision == 1
            break
        end
    end
end

% Evaluate whether the configuration <q> is in collision with a spherical
% obstacle centered at <sphereCenter> with radius <r>.
% input: q -> 1x4 vector of joint angles
%        sphereCenter -> 3x1 vector that denotes sphere center
%        r -> radius of sphere 
% output: collision -> binary number that denotes whether this
%           configuration is in collision or not.
function collision = robotCollision(robot, q, sphereCenter, sphereRadius)
    x1 = [0; 0; 0];
    T = robot.A(1, q) * robot.A(2, q) * robot.A(3, q);
    x2 = T(1 : 3, 4);
    T = T * robot.A(4, q);
    x3 = T(1 : 3, 4);
    T = T * robot.A(5, q);
    x4 = T(1 : 3, 4);
    T = T * robot.A(6, q);
    x5 = T(1 : 3, 4);

    vec = 0 : 0.1 : 1;
    m = size(vec, 2);

    x12 = repmat(x2 - x1, 1, m) .* repmat(vec, 3, 1) + repmat(x1, 1, m);
    x23 = repmat(x3 - x2, 1, m) .* repmat(vec, 3, 1) + repmat(x2, 1, m);
    x34 = repmat(x4 - x3, 1, m) .* repmat(vec, 3, 1) + repmat(x3, 1, m);
    x45 = repmat(x5 - x4, 1, m) .* repmat(vec, 3, 1) + repmat(x4, 1, m);
    x = [x12 x23 x34 x45];
    
    if sum(sum((x - repmat(sphereCenter, 1, size(x, 2))) .^ 2, 1) < sphereRadius^2) > 0
        collision = 1;
    else
        collision = 0;
    end
end

% Smooth path given in qMilestones
% input: robot -> puma 560 robot
%        qMilestones -> nx4 vector of n milestones. 
%        sphereCenter -> 3xN position of center of spherical obstacle
%        sphereRadius -> 1xN radius of obstacle
% output -> qMilestones -> 4xm vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You should output a number of
%                    milestones m<=n.
function qMilestonesSmoothed = smoothMilestones(robot, qMilestones, sphereCenter, sphereRadius)
    [m, ~] = size(qMilestones);
    qMilestonesSmoothed = [];

    while m > 0
        qMilestonesSmoothed = [qMilestones(m, :); qMilestonesSmoothed];
        qGoal = qMilestonesSmoothed(1, :);
        m = m - 1;

        for i = 1: m
            qFar = qMilestones(i, :);
            if checkCollision(robot, qFar, qGoal, sphereCenter, sphereRadius) == 0
                m = i;
                break
            end
        end
    end
end

% creates a trajectory of the path from the milestones
function trajectory = interpolateMilestones(qMilestones)
    d = 0.05;
    trajectory = [];

    for i = 2 : size(qMilestones, 1)        
        delta = qMilestones(i, :) - qMilestones(i - 1, :);
        m = max(floor(norm(delta) / d), 1);
        vec = linspace(0, 1, m);
        leg = repmat(delta', 1, m) .* repmat(vec, size(delta, 2), 1) + repmat(qMilestones(i - 1, :)', 1, m);
        trajectory = [trajectory; leg'];
    end
end