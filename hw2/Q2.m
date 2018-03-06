% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end effector at xGoal.
function qMilestones = Q2(rob, sphereCenter, sphereRadius, qStart, xGoal)
    qGoal = rob.ikine(transl(xGoal), zeros(1, 4), [1, 1, 1, 0, 0, 0]);
    
    adj = zeros(1);
    rTree = qStart;
    qNew = zeros(1, 4);
    i = 0;

    while qNew ~= qGoal
        qRand = qGoal;
        if (mod(i, 10) ~= 0)
            qRand = -pi + 2 * pi * rand(1, 4);
        end

        if (ismember(qRand, rTree, 'rows') == 0)
            node = 1;
            qNear = rTree(1, :);
            qDiff = sum((qRand - qNear) .^ 2);
            [m , ~] = size(rTree);
            for j = 2: m
                qTemp = rTree(j, :);
                qTempDiff = sum((qRand - qTemp) .^ 2);
                if (qTempDiff < qDiff)
                    qDiff = qTempDiff;
                    qNear = qTemp;
                    node = j;
                end
            end

            if (Q1(rob, qNear, qRand, sphereCenter, sphereRadius) == 0)
                qNew = qRand;
                rTree = [rTree; qNew];
                [m, ~] = size(adj);
                adj = [adj; zeros(1, m)];
                adj = [adj zeros(m + 1, 1)];
                [m, ~] = size(rTree);
                adj(m, node) = 1;
                adj(node, m) = 1;
            end
        end

        i = i + 1;
    end

    [m, ~] = size(adj);
    G = graph(adj);
    path = shortestpath(G, 1, m);
    qMilestones = rTree(path, :);
end