% Smooth path given in qMilestones
% input: qMilestones -> nx4 vector of n milestones. 
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xm vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You should output a number of
%                    milestones m<=n.
function qMilestonesSmoothed = Q3(rob, qMilestones, sphereCenter, sphereRadius)
    [m, ~] = size(qMilestones);
    qMilestonesSmoothed = [];

    while m > 0
        qMilestonesSmoothed = [qMilestones(m, :); qMilestonesSmoothed];
        qGoal = qMilestonesSmoothed(1, :);
        m = m - 1;

        for i = 1: m
            qFar = qMilestones(i, :);
            if (Q1(rob, qFar, qGoal, sphereCenter, sphereRadius) == 0)
                m = i;
                break
            end
        end
    end
end
