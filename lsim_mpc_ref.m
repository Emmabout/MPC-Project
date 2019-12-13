function [y, t, x] = lsim_mpc_ref(sys, mpc, x0, xref, duration, Ts, substeps)
N = ceil(duration/Ts);
M = substeps;

n = length(x0);
m = length(sys.y);

x = zeros(n, N*M+1);
y = zeros(m, N*M+1);
t = (0:M*N)*Ts/M;

x(:,1) = x0;

for i = 1:N
    [y_step, ~, x_step] = lsim(sys, [mpc.get_u(x(:,M*(i-1)+1), xref)*ones(1, M) 0], (0:M)*Ts/M, x(:,M*(i-1)+1), 'zoh');
    x(:,M*(i-1)+1:M*i+1) = x_step';
    y(:,M*(i-1)+1:M*i+1) = y_step';
end
end