close all

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
x_ref = -2;
y_ref = -2;
z_ref = -2;
yaw_ref = -pi/4;

[x, t_x, x_x] = lsim_mpc_ref(sys_x, mpc_x, [0 0 0 0], x_ref, 10, Ts, 10);
[y, t_y, x_y] = lsim_mpc_ref(sys_y, mpc_y, [0 0 0 0], y_ref, 10, Ts, 10);
[z, t_z, x_z] = lsim_mpc_ref(sys_z, mpc_z, [0 0], z_ref, 10, Ts, 10);
[yaw, t_yaw, x_yaw] = lsim_mpc_ref(sys_yaw, mpc_yaw, [0 0], yaw_ref, 10, Ts, 10);

%% Plot
% Sys X
figure
% suptitle('System X-Pitch, x0 = [0 0 0 0]')
subplot(2, 2, 1)
hold on
line([t_x(1) t_x(end)], [x_ref x_ref], 'Color', 'Magenta')
p = plot(t_x, x_x(4,:));
grid on
title({''; 'Position'})
xlabel('Time [s]')
ylabel('Position [m]')
% legend(p, 'Position')

subplot(2, 2, 3)
hold on
line([t_x(1) t_x(end)], [0 0], 'Color', 'Black')
p = plot(t_x, x_x(3,:));
grid on
title({''; 'Velocity'})
xlabel('Time [s]')
ylabel('Velocity [m/s]')
% legend(p, 'Velocity')

subplot(2, 2, 2)
hold on
line([t_x(1) t_x(end)], [0 0], 'Color', 'Black')
p = plot(t_x, x_x(2,:));
grid on
title({''; 'Pitch'})
xlabel('Time [s]')
ylabel('Pitch [rad]')
% legend(p, 'Pitch')

subplot(2, 2, 4)
hold on
line([t_x(1) t_x(end)], [0 0], 'Color', 'Black')
p = plot(t_x, x_x(1,:));
grid on
title({''; 'Pitch-Rate'})
xlabel('Time [s]')
ylabel('Pitch-Rate [rad/s]')
% legend(p, 'Pitch-Rate')

%%
% Sys Y
figure
% suptitle('System X-Pitch, x0 = [0 0 0 0]')
subplot(2, 2, 1)
hold on
line([t_y(1) t_y(end)], [y_ref y_ref], 'Color', 'Magenta')
p = plot(t_y, x_y(4,:));
grid on
title({''; 'Position'})
xlabel('Time [s]')
ylabel('Position [m]')
% legend(p, 'Position')

subplot(2, 2, 3)
hold on
line([t_y(1) t_y(end)], [0 0], 'Color', 'Black')
p = plot(t_y, x_y(3,:));
grid on
title({''; 'Velocity'})
xlabel('Time [s]')
ylabel('Velocity [m/s]')
% legend(p, 'Velocity')

subplot(2, 2, 2)
hold on
line([t_y(1) t_y(end)], [0 0], 'Color', 'Black')
p = plot(t_y, x_y(2,:));
grid on
title({''; 'Roll'})
xlabel('Time [s]')
ylabel('Roll [rad]')
% legend(p, 'Roll')

subplot(2, 2, 4)
hold on
line([t_y(1) t_y(end)], [0 0], 'Color', 'Black')
p = plot(t_y, x_y(1,:));
grid on
title({''; 'Roll-Rate'})
xlabel('Time [s]')
ylabel('Roll-Rate [rad/s]')
% legend(p, 'Roll-Rate')

%%
% Sys Z
figure
% suptitle('System Z, x0 = [0 0]')
subplot(2, 1, 1)
hold on
line([t_z(1) t_z(end)], [z_ref z_ref], 'Color', 'Magenta')
p = plot(t_z, x_z(2,:));
grid on
title({''; 'Position'})
xlabel('Time [s]')
ylabel('Position [m]')
% legend(p, 'Position')

subplot(2, 1, 2)
hold on
line([t_z(1) t_z(end)], [0 0], 'Color', 'Black')
p = plot(t_z, x_z(1,:));
grid on
title({''; 'Velocity'})
xlabel('Time [s]')
ylabel('Velocity [m/s]')
% legend(p, 'Velocity')


%%
%Sys yaw
figure
% suptitle('System Yaw, x0 = [0 0]')
subplot(2, 1, 1)
hold on
line([t_yaw(1) t_yaw(end)], [yaw_ref yaw_ref], 'Color', 'Magenta')
p = plot(t_yaw, x_yaw(2,:));
grid on
title({''; 'Yaw'})
xlabel('Time [s]')
ylabel('Yaw [rad]')
% legend(p, 'Yaw')

subplot(2, 1, 2)
hold on
line([t_yaw(1) t_yaw(end)], [0 0], 'Color', 'Black')
p = plot(t_yaw, x_yaw(1,:));
grid on
title({''; 'Yaw-Rate'})
xlabel('Time [s]')
ylabel('Yaw-Rate [rad/s]')
% legend(p, 'Yaw-Rate')