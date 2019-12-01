%% Init
quad = Quad();
[xs,us] = quad.trim();        % Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us); % Linearize the nonlinear model

systransformed = sys*inv(quad.T);  % New system is  A*x + B*inv(T)*v

[sysx, sysy, sysz, sysyaw] = quad.decompose(sys, xs, us);


%% system Z 
% System parameters
N = 10;
Q = 10 * eye(2);
R = 1;

A = sysz.A;
B = sysz.B;

x = sdpvar(2,N,'full');
u = sdpvar(1,N-1,'full');

% Constraints
M  = [1; -1];
m = [0.3; 0.2];

% Define constraints and objective
con = [];
obj = 0;

for i = 1:N-1
    con = con + (x(:,i+1) == A*x(:,i) + B*u(:,i));  % System dynamics
    con = [con, M*u(:,i)<= m];                       % Input constraints
%     con = [con, F*x(:,i)<= f];                       % State constraints
    obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i);    % Cost function
end
% con = con + (Ff*x(:,N) <= ff);
obj = obj + x(:,N)'*Q*x(:,N);

% Compile the matrices
ctrl = optimizer(con, obj, sdpsettings('solver','sedumi'), x(:,1), u(:,1));


%% system yaw