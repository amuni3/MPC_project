% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_1_forces(T)
% controller variables
persistent param forces_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, forces_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
    T_input = T - param.T_sp; % X(k) == Xo
    [u_mpc,errorcode] = forces_optimizer{T_input};
    if (errorcode ~= 1)
        warning('MPC infeasible');
    end
    % Implement only the first input from the MPC input sequence
    p = u_mpc + param.p_sp;
end

function [param, forces_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

param = compute_controller_base_parameters; % get basic controller parameters

%% implement your MPC using Yalmip2Forces interface here, e.g.
N = 30;
nx = size(param.A,1); %3
nu = size(param.B,2); %2

U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full'); % 2*(N-1)
X = sdpvar(repmat(nx,1,N),ones(1,N),'full'); % 3*(N)

xmin = param.Xcons(:,1);
xmax = param.Xcons(:,2);
umin = param.Ucons(:,1);
umax = param.Ucons(:,2);

objective = 0;
constraints = [];

objective = objective + X{:,1}' * param.Q * X{:,1} + ...
                        U{:,1}' * param.R * U{:,1};
constraints = [X{:,2} == param.A * X{:,1} + param.B * U{:,1}, ...
               umin <= U{:,1} <= umax];
for k = 2:N-1
  constraints = [constraints, ...
                 X{:,k+1} == param.A * X{:,k} + param.B * U{:,k}, ...
                 xmin <= X{:,k} <= xmax, umin <= U{:,k} <= umax];                
  objective = objective + ...
              X{:,k}' * param.Q * X{:,k} + U{:,k}' * param.R * U{:,k};
end

% Terminal State Constraint
[P,~,~] = dare(param.A, param.B, param.Q, param.R);
objective = objective + X{:, N}' * P * X{:, N}; 

parameters_in = X{1,1}; % Input to the system is x(k) which is Xo (constraint)
solutions_out = U{1,1}; % Only care about the first input in input sequence

codeoptions = getOptions('FORCESNLPsolver');
codeoptions.optlevel = 2;
codeoptions.timing = 1;
codeoptions.printlevel = 2;
forces_optimizer = optimizerFORCES(constraints, objective, codeoptions, parameters_in, solutions_out);
end