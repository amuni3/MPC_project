% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    Ucons = param.Ucons;
    Xcons = param.Xcons;

    %% Compute the invariant set
    % computes a control invariant set using LQR controller
    [~,~,G] = dare(param.A,param.B,param.Q,param.R);
    K = - G;
    systemLQR = LTISystem('A', param.A + param.B * K);
    % MPC Lec 5 slide 43
    A_x = [eye(3); -eye(3); K; -K];
    b_x = [Xcons(:,2);-1 * Xcons(:,1); ...
           Ucons(:,2);-1 * Ucons(:,1)];
    Xp = Polyhedron('A', A_x, 'b', b_x);                      
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;    
    InvSetLQR = systemLQR.invariantSet()  
    % InvSetLQR.plot(), alpha(0.25)
    
    %% Calculate Invariant set --> Commputationally heavy
    % system = LTISystem('A', param.A, 'B', param.B);
    % system.x.min = Xcons(:,1);
    % system.x.max = Xcons(:,2);
    % system.u.min = Ucons(:,1);
    % system.u.max = Ucons(:,2);
    % InvSet = system.invariantSet();
    % plot(InvSet), alpha(0.25), ...
    % title('Control Invariant Set for Truck')    
end

