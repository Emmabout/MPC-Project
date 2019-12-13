%% Init
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

%% Simulate the systems
[x, t_x, x_x] = lsim_mpc_ref(sys_x, mpc_x, [0 0 0 0], -2, 10, Ts, 10);

%% Plot
% Sys X
figure
suptitle('System X-Pitch, x0 = [0 0 0 2]')
subplot(2, 2, 1)
hold on
line([t_x(1) t_x(end)], [0 0], 'Color', 'Black')
p = plot(t_x, x_x(4,:));
grid on
title({''; 'Position'})
xlabel('Time [s]')
ylabel('Position [m]')
legend(p, 'Position')

subplot(2, 2, 3)
hold on
line([t_x(1) t_x(end)], [0 0], 'Color', 'Black')
p = plot(t_x, x_x(3,:));
grid on
title({''; 'Velocity'})
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend(p, 'Velocity')

subplot(2, 2, 2)
hold on
line([t_x(1) t_x(end)], [0 0], 'Color', 'Black')
p = plot(t_x, x_x(2,:));
grid on
title({''; 'Pitch'})
xlabel('Time [s]')
ylabel('Pitch [rad]')
legend(p, 'Pitch')

subplot(2, 2, 4)
hold on
line([t_x(1) t_x(end)], [0 0], 'Color', 'Black')
p = plot(t_x, x_x(1,:));
grid on
title({''; 'Pitch-Rate'})
xlabel('Time [s]')
ylabel('Pitch-Rate [rad/s]')
legend(p, 'Pitch-Rate')