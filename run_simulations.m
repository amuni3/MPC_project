% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 
% execute simulation starting from T0_1 using lqr controller with scenario1
param = compute_controller_base_parameters;
T0_1 = param.T_sp + [3; 1; 0];
T0_2 = param.T_sp + [-1; -0.1; -4.5];

%% Question 5
 [T, p] = simulate_truck(T0_1, @controller_lqr, scen1);

%% Question 7
 [T, p] = simulate_truck(T0_2, @controller_lqr, scen1);

%% Question 9
 [T, p] = simulate_truck(T0_1, @controller_mpc_1, scen1);
 [T, p] = simulate_truck(T0_2, @controller_mpc_1, scen1);

%% Question 12
 [T, p] = simulate_truck(T0_1, @controller_mpc_2, scen1);

%% Question 13 
 [T, p] = simulate_truck(T0_2, @controller_mpc_2, scen1);

%% Question 15
 [T, p] = simulate_truck(T0_1, @controller_mpc_3, scen1);
 [T, p] = simulate_truck(T0_2, @controller_mpc_3, scen1);

%% Question 17
 The following is infeasible
 [T, p] = simulate_truck(T0_2, @controller_mpc_3, scen2);

%% Questionn 18
 [T, p] = simulate_truck(T0_2, @controller_mpc_4, scen2);

%% Question 19
 [T, p] = simulate_truck(T0_2, @controller_mpc_3, scen1);
 [T, p] = simulate_truck(T0_2, @controller_mpc_4, scen1);

%% Question 22
global x_hat
[T, p] = simulate_truck(T0_1, @controller_mpc_5, scen3);
% Disturbance plot
figure(2);
hold on;
plot(x_hat(4,:));
plot(x_hat(5,:));
plot(x_hat(6,:));
legend 'd_1' 'd_2' 'd_3';

%% Question 23
disp('Running MPC1 with Forces')
% Run once in order to not consider "initilization time"
[T,p,t_sim_forces] = simulate_truck(T0_2, @controller_mpc_1_forces, scen1);
sim_forces = 0;
sim = 0;
for i=1:20
    [T,p,t_sim_forces] = simulate_truck(T0_2, @controller_mpc_1_forces, scen1);
    [T,p,t_sim] = simulate_truck(T0_2, @controller_mpc_1, scen1);
    sim_forces = sim_forces + t_sim_forces;
    sim = sim + t_sim;
end
disp('Simulation times')
disp(t_sim_forces/20);
disp(t_sim/20);

%% Plots
% lines=findobj(gcf,'Type','Line');
% for i=1:numel(lines)
%     lines(i).LineWidth=1.5;    
% end    
% 
% print('Q22_Disturbance_Estimate',...
%     '-dpdf','-fillpage')
