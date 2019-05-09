% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_5(T)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();

end

%% Calculate observer value


%% evaluate control action by solving MPC problem, e.g.
T_input = T - param.T_sp;
[u_mpc,errorcode] = yalmip_optimizer{T_input};
if (errorcode ~= 0)
      warning('MPC infeasible');
end
p = u_mpc{1} + param.p_sp;
end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

param = compute_controller_base_parameters; % get basic controller parameters
param.d_hat = 0;

%% implement your MPC using Yalmip here, e.g.
N = 30;

% One extra state variable for the disturbance.
nx = size(param.A,1) + 3;
nu = size(param.B,2);

U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');


Q_aug = [param.Q, zeros(3,3);
         zeros(3,3), zeros(3,3)];
R_aug = [param.R];
xmin = param.Xcons(:,1);
xmax = param.Xcons(:,2);
umin = param.Ucons(:,1);
umax = param.Ucons(:,2);

% initialization
objective = 0;
constraints = [];

% Cost of x0
objective = objective + ...
            X{:,1}' * Q_aug * X{:,1} + U{:,1}' * R_aug * U{:,1};

% Constraints for x1
x0 = X{:,1}
x1 = X{:,2}
constraints = [...
    % Dynamics constraint
    x1 == A_aug*x0 + B_aug*u, ...
    % Input constraint
    umin <= U{:,1} <= umax,...
    ];

for k = 1:N-1
    constraints = [constraints, ...
        X{:,k+1} == param.A * X{:,k} + param.B * U{:,k},...
        xmin <= X{:,k} <= xmax, umin <= U{:,k} <= umax];                


    constraints = [...
        % Dynamics constraint
        x{:,k+1} == A_aug*x{:,k} + B_aug*u, ...
        % State constraint
        xmin <= X{1:3,k} <= xmax,
        % Input constraint
        umin <= U{:,k} <= umax,...
        ];
    objective = objective + ...
        X{:,k}' * Q_aug * X{:,k} + U{:,k}' * R_aug * U{:,k};
end
% objective = objective + ... ;
%
% ops = sdpsettings('verbose',0,'solver','quadprog');
% fprintf('JMPC_dummy = %f',value(objective));
% yalmip_optimizer = optimizer(constraints,objective,ops,... , ... );


A_aug = [param.A,    param.Bd;
         zeros(3,3), eye(3,3)];
B_aug = [param.B; zeros(3,2)];
C_aug = [eye(3,3), zeros(3,3)];

param.A_aug = A_aug
param.B_aug = B_aug
param.C_aug = C_aug
end
