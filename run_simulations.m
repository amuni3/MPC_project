% Init
clear all
close all
clc
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
%[T, p] = simulate_truck(T0_1, @controller_lqr,scen1);

%% Question 9
%[T, p] = simulate_truck(T0_1, @controller_mpc_1,scen1);
% [T, p] = simulate_truck(T0_2, @controller_mpc_1,scen1);

%% Question 12
%[T, p] = simulate_truck(T0_1, @controller_mpc_2,scen1);

%% Question 18
%[T, p] = simulate_truck(T0_2, @controller_mpc_4,scen2);

%% Question 19
%[T, p] = simulate_truck(T0_2, @controller_mpc_3,scen1);
%[T, p] = simulate_truck(T0_2, @controller_mpc_4,scen1);
%%

% lines=findobj(gcf,'Type','Line');
% for i=1:numel(lines)
%     lines(i).LineWidth=2.5;
% end    

% print('Q19_SimulationMPC4','-dpdf','-fillpage')





%% Question 
% [T, p] = simulate_truck(T0_1, @controller_mpc_5, scen3);

% lines=findobj(gcf,'Type','Line');
% for i=1:numel(lines)
%     lines(i).LineWidth=2.5;
% end    
% print('Q22_SimulationMPC5','-dpdf','-fillpage')

[T, p] = simulate_truck(T0_1, @controller_mpc_3, scen3);

lines=findobj(gcf,'Type','Line');
for i=1:numel(lines)
    lines(i).LineWidth=2.5;
end

lines=findobj(gcf,'Type','Line');
for i=1:numel(lines)
    lines(i).LineWidth=2.5;
end
print('Q22_SimulationMPC3','-dpdf','-fillpage')
