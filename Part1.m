quad = Quad();
Tf = 1.0; % Time to simulate for

x0 = zeros(12,1);                                % Initial state
u = [0;1;0;1];                                   % Input to apply
sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0);  % Solve the system ODE
quad.plot(sim, u);


