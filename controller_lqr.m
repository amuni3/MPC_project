% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_lqr(T)
% controller variables
persistent param;

% initialize controller, if not done already
if isempty(param)
    param = init();
end

% compute control action
% F_inf = param.A' * P_inf * param.A + param.Q - param.A' * ... 
%        P_inf * param.B * inv(param.B' * P_inf * param.B + param.R) * ...
%        param.B' * P_inf * param.A

if ((param.counter == 30) && (norm(param.T_sp - T) < 0.2*norm(param.xo)))
    disp(T);
    disp('Norm conditions satisfied!!')
end

% X --> P_inf % G --> F_inf
% u(k) = F_inf * x(k)
T_in = (T - param.T_sp);
p = (param.F * T_in) + param.p_sp; 

% Infinite horizon cost - is a Lyapunov function for the system
% J_inf  = x'*P_inf*x
if (param.counter == 1)
    J_inf = (T - param.T_sp)' * param.P * (T - param.T_sp);
    disp(J_inf);
end
param.counter = param.counter + 1;
end

function param = init()
param = compute_controller_base_parameters;
% add additional parameters if necessary, e.g.
param.xo = [3; 1; 0];
param.counter = 1;
[X,~,G] = dare(param.A,param.B,param.Q,param.R);
param.F = - G;
param.P = X;
end