% TODO: You write this function!
% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q2(f,qInit,posGoal)
    q = qInit;
    A = 0.01;
    count = 0;

    while count < 500
        count = count + 1;
        T = f.fkine(q);
        Xerr = posGoal - transpose(transl(T));
        Jv = f.jacob0(q, 'trans');
        Dq = pinv(Jv) * A * Xerr;
        q = q + transpose(Dq);
    end
end