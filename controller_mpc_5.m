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
global x_hat

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init;
    param.x_hat(1:3) = T;
    x_hat = [x_hat];
end

%% Calculate current observer value


% Calculate new reference.
reference = [eye(3) - param.A, -param.B; 
             param.H,          zeros(2,2)] \ ...
            [param.x_hat(4:6);
             param.r];

%% evaluate control action by solving MPC problem, e.g.
T_input = T - reference(1:3);
[u_mpc,errorcode] = yalmip_optimizer{T_input, reference(1:3), reference(4:5)};
if (errorcode ~= 0)
      warning('MPC infeasible');
end
p = u_mpc{1} + reference(4:5);

param.x_hat = param.A_aug * param.x_hat ...
    + param.B_aug * p ...
    + param.L * param.C_aug * [(T - param.x_hat(1:3)); zeros(3,1)];
x_hat = [x_hat, param.x_hat];
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

X_offset = sdpvar(nx,1);
U_offset = sdpvar(nu,1);


Ucons = param.Pcons - [U_offset, U_offset];
Xcons = param.Tcons - [X_offset, X_offset];
xmin = Xcons(:,1);
xmax = Xcons(:,2);
umin = Ucons(:,1);
umax = Ucons(:,2);

objective = 0;
constraints = [];

objective = objective + ...
            X{:,1}' * param.Q * X{:,1} + U{:,1}' * param.R * U{:,1};
constraints = [X{:,2} == param.A * X{:,1} + param.B * U{:,1}, ...
               umin <= U{:,1} <= umax];
for k = 2:N-1
  constraints = [constraints, ...
                 X{:,k+1} == param.A * X{:,k} + param.B * U{:,k},...
                 xmin <= X{:,k} <= xmax, umin <= U{:,k} <= umax];                
  objective = objective + ...
              X{:,k}' * param.Q * X{:,k} + U{:,k}' * param.R * U{:,k};
end

% Terminal Set State Constraint
[A_x, b_x] = compute_X_LQR;
constraints = [constraints, A_x * X{:,N} <= b_x];

% Terminal Input Constraint
[P,~,~] = dare(param.A, param.B, param.Q, param.R);
objective = objective + X{:, N}' * P * X{:, N}; 

% Input to the system is x(k) which is Xo (constraint)
parameters_in = {X{1,1}, X_offset, U_offset}; 
solutions_out = {U{1,1}, objective}; % Only care about the first input in input sequence
ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_optimizer = optimizer(constraints, objective, ops, parameters_in, solutions_out);

A_aug = [param.A,    eye(3,3);
         zeros(3,3), eye(3,3)];
B_aug = [param.B; zeros(3,2)];
C_aug = [eye(3,3), zeros(3,3)];

param.A_aug = A_aug;
param.B_aug = B_aug;
param.C_aug = C_aug;

% Calculate observer gains
param.L = place(A_aug', C_aug', [0; 0; 0; 0.4; 0.4; 0.4])';
param.p = [0; 0];

% Initialize the observer with an estimated disturbance value.
param.x_hat = [0; 0; 0; param.Bd];
end