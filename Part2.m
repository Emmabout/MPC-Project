quad = Quad();
[xs,us] = quad.trim();        % Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us); % Linearize the nonlinear model

systransformed = sys*inv(quad.T);  % New system is  A*x + B*inv(T)*v

[sysx, sysy, sysz, sysyaw] = quad.decompose(sys, xs, us);