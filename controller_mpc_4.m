% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_4(T)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
T_input = T - param.T_sp;  % X(k) == Xo
    [u_mpc,errorcode] = yalmip_optimizer{T_input};
    if (errorcode ~= 0)
        warning('MPC infeasible');
    end
    % Implement only the first input from the MPC input sequence
    p = u_mpc{1} + param.p_sp;
    % disp(u_mpc{3});   
end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

param = compute_controller_base_parameters; % get basic controller parameters

%% implement your MPC using Yalmip here, e.g.
N = 30;
nx = size(param.A,1); %3
nu = size(param.B,2); %2

U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full'); % 2*(N-1)
X = sdpvar(repmat(nx,1,N),ones(1,N),'full'); % 3*(N)
% Add slack variables for soft constrains
% (Sec. 6): https://yalmip.github.io/debugginginfeasible/   
% NOTE: You have the same "leeway" on lower AND upper bound because
% at a time, your system will only be violating ONE of the bounds.
slack_x = sdpvar(nx,1);

xmin = param.Xcons(:,1);
xmax = param.Xcons(:,2);
umin = param.Ucons(:,1);
umax = param.Ucons(:,2);

objective = 0;
% Slack variable constraint
constraints = [slack_x >= 0];

objective = objective + ...
            X{:,1}' * param.Q * X{:,1} + U{:,1}' * param.R * U{:,1};
constraints = [X{:,2} == param.A * X{:,1} + param.B * U{:,1}, ...
               umin <= U{:,1} <= umax];
for k = 2:N-1
  constraints = [constraints, ...
                 X{:,k+1} == param.A * X{:,k} + param.B * U{:,k},...
                 xmin - slack_x <= X{:,k} <= xmax + slack_x, ...
                 umin <= U{:,k} <= umax];                
  objective = objective + ...
              X{:,k}' * param.Q * X{:,k} + U{:,k}' * param.R * U{:,k};
end

% Slack Penalty (penalize constraint violation)
% Need both linear and quadratic cost to attain exact penalty
S = [10, 0, 0;
     0, 10, 0;
     0, 0, 0];
v = 10^6; %WTF
objective = objective + slack_x' * S * slack_x + v * norm(slack_x, 1);

% Terminal Set State Constraint
[A_x, b_x] = compute_X_LQR;
constraints = [constraints, A_x * X{:,N} <= b_x];

% Terminal Input Constraint
[P,~,~] = dare(param.A, param.B, param.Q, param.R);
objective = objective + X{:, N}' * P * X{:, N}; 

parameters_in = X{1,1}; % Input to the system is x(k) which is Xo (constraint)
solutions_out = {U{1,1}, objective, slack_x}; % Only care about the first input in input sequence
ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_optimizer = optimizer(constraints, objective, ops, parameters_in, solutions_out);
end