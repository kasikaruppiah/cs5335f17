% **************************************************************
% ************** You don't need to modify this function.
% **************************************************************
function [xmat,umat] = Q1(A,B,QT,Q,R,T,x0)

    % calculate value function
    Pseq = FH_DT_Riccati(A,B,QT,Q,R,T);

    % integrate forward the vehicle dynamics
    [xmat,umat] = integrate_dynamics(A,B,R,x0,Pseq,T);

end

% **************************************************************
% ************** You don't need to modify this function.
% **************************************************************
% Get control action and integrate dynamics foward one step in time. This
% function calls <getControl>, a function that you must implement. 
% input: A,B,R -> parameters of system and cost function
%        x0 -> 4x1 initial state
%        Pseq -> cell array of 4x4 P matrices. Cell array goes from 1 to T
%        T -> time horizon
% output: xmat -> 4xT matrix of state as a function of time
%         umat -> 2x(T-1) matrix of control actions as a function of time
function [xmat, umat] = integrate_dynamics(A,B,R,x0,Pseq,T)
    i=0;
    x = x0;
    for i=1:T
        
        % get control action
        u = getControl(A,B,R,Pseq,x,i);

        % integrate dynamics forward one time step
        x = A*x + B*u;
        
        xmat(:,i) = x;
        umat(:,i) = u;

    end
end

% Calculate control action to take from the current state.
% input: A,B,R -> parameters of system and cost function
%        P -> P matrix for this time step
%        x -> current state
%        i -> current time step of controller
% output: u -> control action to take
function u = getControl(A, B, R, Pseq, x, i)
    u = - inv(R + B.' * Pseq{i} * B) * B.' * Pseq{i} * A * x;
end

% Calculate time-varying value function using riccati eqn. (i.e.
% compute the sequence of P matrices.)
% input: A,B,Qf,Q,R -> parameters of dynamics and cost function
%        T -> time horizon
% output: Pseq -> cell array of 4x4 P matrices. Cell array goes from 1 to T
function Pseq = FH_DT_Riccati(A, B, Qf, Q, R, T)
    Pseq = {Qf};
    for i = T : -1 : 2
        Pt = Q + A.' * Pseq{1} * A - A.' * Pseq{1} * B * inv(R + B.' * Pseq{1} * B) * B.' * Pseq{1} * A;
        Pseq = [Pt Pseq];
    end
end