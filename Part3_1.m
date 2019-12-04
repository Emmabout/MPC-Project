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

% Get control inputs with
% x = [0 0.03 0 0]';
% y = [0 0.03 0 0]';
% z = [0 2]';
% yaw = [0 pi/8]';

[x, t_x, x_x] = lsim_mpc(sys_x, mpc_x, [0 0 0 2], 8, Ts, 10);
[y, t_y] = lsim_mpc(sys_y, mpc_y, [0 0 0 2], 8, Ts, 10);
[z, t_z] = lsim_mpc(sys_z, mpc_z, [0 2], 8, Ts, 10);
[yaw, t_yaw] = lsim_mpc(sys_yaw, mpc_yaw, [0 pi/4], 8, Ts, 10);

figure
plot(t_x, x_x)

figure
subplot(4, 1, 1)
plot(t_x, x)
title('X Position Over Time')
xlabel('Time [s]')
ylabel('X [m]')

subplot(4, 1, 2)
plot(t_y, y)
title('Y Position Over Time')
xlabel('Time [s]')
ylabel('Y [m]')

subplot(4, 1, 3)
plot(t_z, z)
title('Z Position Over Time')
xlabel('Time [s]')
ylabel('Z [m]')

subplot(4, 1, 4)
plot(t_yaw, yaw)
title('Yaw Over Time')
xlabel('Time [s]')
ylabel('Yaw [rad]')