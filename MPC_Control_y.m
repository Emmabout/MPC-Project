classdef MPC_Control_y < MPC_Control
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opt = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x(:,1) - initial state (estimate)
            %   xs, us - steady-state target
            % OUTPUTS
            %   u(:,1) - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            [n,m] = size(mpc.B);
            
            % Steady-state targets (Ignore this before Todo 3.2)
            xs = sdpvar(n, 1);
            us = sdpvar(m, 1);
            
            % SET THE HORIZON HERE
            N = 30;
            
            % Predicted state and input trajectories
            x = sdpvar(n, N);
            u = sdpvar(m, N-1);
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
            con = [];
            obj = 0;
            
            % Tuning parameters
            Q = diag([10 10 10 100]);
            R = 1;
            
            % Constraints
            F = [0 1 0 0; 0 -1 0 0];
            f = [0.035; 0.035];
            M  = [1; -1];
            m = [0.3; 0.3];
            
            % Compute LQR controller for unconstrained system
            [K,Qf,~] = dlqr(mpc.A, mpc.B, Q, R);
            % MATLAB defines K as -K, so invert its signal
            K = -K;
            
            % Compute maximal invariant set
            Xf = polytope([F;M*K],[f;m]);
            Acl = [mpc.A+mpc.B*K];
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, preXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff,ff] = double(Xf);
            
            for i = 1:N-1
                con = con + (x(:,i+1) == mpc.A*x(:,i) + mpc.B*u(:,i));  % System dynamics
                con = [con, M*u(:,i)<= m];                       % Input constraints
                con = [con, F*x(:,i)<= f];                       % State constraints
                obj = obj + (x(:,i) - xs)'*Q*(x(:,i) - xs) + (u(:,i) - us)'*R*(u(:,i) - us);    % Cost function
            end
            con = con + (Ff*(x(:,N) - xs) <= ff);
            P = dlyap(Acl, Q);
            obj = obj + (x(:,N) - xs)'*P*(x(:,N) - xs);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
            ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x(:,1), xs, us}, u(:,1));
        end
        
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opt = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state targets
            n = size(mpc.A,1);
            xs = sdpvar(n, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            con = [];
            obj = 0;
            
            F = [0 1 0 0; 0 -1 0 0];
            f = [0.035; 0.035];
            M  = [1; -1];
            m = [0.3; 0.3];
            
            con = con + (xs == mpc.A*xs + mpc.B*us); % System dynamics: x(k+1) = x(k)
            con = con + (ref == mpc.C*xs + mpc.D*us); % Output = ref
            con = [con, M*us <= m]; % Input constraints
            con = [con, F*xs <= f]; % State constraints
            
            obj = obj + us^2; % Minimize input
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
            
        end
    end
end
