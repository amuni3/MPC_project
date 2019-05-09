function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_truck');
    
    %% (1) continuous
    Ac = [-((truck.a1o + truck.a12)/truck.m1), truck.a12/truck.m1, 0;
          truck.a12/truck.m2, -((truck.a12+truck.a23+truck.a2o)/truck.m2), ...
          truck.a23/truck.m2;
          0, truck.a23/truck.m3, -((truck.a23+truck.a3o)/truck.m3)];
    
    Bc = [1/truck.m1, 0;
          0, 1/truck.m2;
          0, 0];
    
    Bcd = [1/truck.m1, 0, 0;
           0, 1/truck.m2, 0;
           0, 0, 1/truck.m3];
    
    dc = [truck.a1o * truck.To;
          truck.a2o * truck.To;
          truck.a3o * truck.To;] + truck.w;
      
    %% (2) Discretization
    Ts = 60;
    
    % Exact Discretization (linear system)(exponential of matrix)    
    sys = ss(Ac, Bc, eye(3), []); 
    sysd = c2d(sys, Ts);
    A = sysd.A;
    B = sysd.B;
    % Explicitly take disturbance into account and discretize
    Bd = Ts * Bcd * dc;
       
    % Euler Discretization (This is NOT exact)
    % Use exponential functions since its a linear system
    % A = eye(3) + Ts * Ac
    % B = Ts * Bc;
    % Bd = Ts * Bcd * dc
    
    %% (3) Define symbals for Temperature and Power input steady state
    % This is the reference to be tracked (i.e desired temperatures)
    t_ss1 = sym('t_ss1');
    t_ss2 = sym('t_ss2');
    t_ss3 = sym('t_ss3');
    u_ss1 = sym('u_ss1');
    u_ss2 = sym('u_ss2');
    % NOTE: Your "set points/ steady state values" MAY differ from 
    % reference values (Your system maybe ill-posed, in which case,
    % apply least squares to get the closest points.)
    % So first, independently solve the steady state values, and 
    % hopefully they match the reference values.
    T_ref = [-20; 0.25];
    x_ss = [t_ss1; t_ss2; t_ss3];
    H = [1, 0, 0; 0, 1, 0];
    r = T_ref;
    M = vpa(([eye(3)-A, -B; H, zeros(2,2)] * [x_ss; u_ss1; u_ss2]),5);
    eqn1 = M(1,:) == Bd(1);
    eqn2 = M(2,:) == Bd(2);
    eqn3 = M(3,:) == Bd(3);
    eqn4 = M(4,:) == r(1);
    eqn5 = M(5,:) == r(2);
    sol1 = solve([eqn1, eqn2, eqn3, eqn4, eqn5], ...
                 [t_ss1, t_ss2, t_ss3, u_ss1, u_ss2]);

    % Alternate method:
    % Using pseudo inverse (here its not pseudo cause M is full rank)
    T_ref = [-20; 0.25]; % [t_ss1; t_ss2]
    H = [1, 0, 0; 0, 1, 0];
    r = T_ref;
    M = ([eye(3)-A, -B; H, zeros(2,2)]);
    sol2 = inv(M) * [Bd; r];    
    T_sp = sol2(1:3);
    p_sp = sol2(4:5);

    % Shift the system to center at steady-state points (Delta formultion)
    % (x(k+1) - x_ss) = A (x(k) - x_ss) + B (u(k) - u_ss)
    % x(k+1) = A * x(k) + B * u(k) + Bd
       
    % (3) set point computation
    % T_sp = subs(vpa([sol1.t_ss1; sol1.t_ss2; sol1.t_ss3],5));
    % p_sp = subs([vpa(sol1.u_ss1, 5); vpa(sol1.u_ss2, 5)]);
    
    %% (4) system constraints
    % Note: Discrepancy b/w state constraints in description & struct
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = Pcons - [p_sp, p_sp];
    Xcons = Tcons - [T_sp, T_sp];
    
    %% (5) LQR cost function
    Q = [3000, 0, 0;
         0, 1500, 0;
         0, 0, 0];
    R = 0.05 * eye(2);
    
    %% put everything together
    param.A = A;
    param.B = B;
    param.Q = Q;
    param.R = R;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
end

