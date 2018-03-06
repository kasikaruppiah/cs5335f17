% TODO: You write this function!
% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First six joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)
function q = Q3(f1,f2,qInit,f1Target,f2Target)
    q = qInit;
    A = 0.01;
    count = 0;

    while count < 500
        count = count + 1;
        q1 = q(:, (1 : 9));
        q2 = q(:, [1 : 7, end - 1 : end]);

        T1 = f1.fkine(q1);
        Xerr1 = f1Target - transpose(transl(T1));
        Jv1 = f1.jacob0(q1, 'trans');
        Dq1 = pinv(Jv1) * A * Xerr1;

        T2 = f2.fkine(q2);
        Xerr2 = f2Target - transpose(transl(T2));
        Jv2 = f2.jacob0(q2, 'trans');
        Dq2 = pinv(Jv2) * A * Xerr2;

        Dq = vertcat((Dq1(1 : 7, :) + Dq2(1 : 7, :)) / 2, Dq1(end - 1 : end, :), Dq2(end - 1 : end, :));
        q = q + transpose(Dq);
    end
end

    
