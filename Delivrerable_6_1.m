quad = Quad();
CTRL = ctrl_NMPC(quad);
sim = quad.sim(CTRL)
quad.plot(sim)

function [ctrl, traj] = ctrl_NMPC(quad)
import casadi.*
opti = casadi.Opti(); % Optimization problem
N = 20; % MPC horizon [SET THIS VARIABLE]
% ???? decision variables ?????????
X = opti.variable(12,N+1); % state trajectory variables
U = opti.variable(4, N); % control trajectory (throttle, brake)
X0 = opti.parameter(12,1); % initial state
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE HERE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%
% Tuning parameters
Q = diag([10 10 10, 10 10 100, 10 10 10, 100 100 100]);
R = eye(4);

% Reference state and command
XS = [0 0 0, 0 0 REF(4), 0 0 0, REF(1:3)']';
[~, US] = quad.trim();

% Initial conditions
opti.subject_to(X(:,1) == X0);

obj_fun = 0; % Objective function
for k=1:N % loop over control intervals
    opti.subject_to(X(:,k+1) == X(:,k) + quad.Ts*quad.f(X(:,k), U(:,k))); % System dynamics
    obj_fun = obj_fun + (X(:,k) - XS)'*Q*(X(:,k) - XS) + (U(:,k) - US)'*R*(U(:,k) - US); % State and command cost
end

opti.subject_to(0 <= U <= 1.5) % Input constraints
opti.minimize(obj_fun)

ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end

function u = eval_ctrl(x, ref, opti, X0, REF, X, U)
% ???? Set the initial state and reference ????
opti.set_value(X0, x);
opti.set_value(REF, ref);
% ???? Setup solver NLP ??????
ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);
% ???? Solve the optimization problem ????
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');
u = opti.value(U(:,1));
% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end