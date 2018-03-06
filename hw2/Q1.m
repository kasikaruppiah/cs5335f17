% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        sphereCenter -> 3x1 position of center of sphere
%        r -> radius of sphere
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = Q1(rob, q1, q2, sphereCenter, r)
    points = 11;
    % delta q - increments for evenly spaced points between q1 and q2
    qDelta = (q2 - q1) / points;
    % Total 12 points
    % q1, 10 points between q1 and q2, q2
    for i = 0:points
        qRand = q1 + i * qDelta;
        collision = robotCollision(rob, qRand, sphereCenter, r);
        if collision == 1
            break
        end
    end
end