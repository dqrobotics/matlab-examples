%% Example of torque actuation and reading
% Frederico Fernandes Afonso Silva - Jul/2023
% Last modification: Aug/2024

% Example of torque actuation and reading using a KUKA LBR4 robot.
%
% Usage:
%       1) Open scene "lbr4_torque_control.ttt" on V-REP.
%           Available at: https://osf.io/gmhba/?view_only=e0122282979d43af93aa280cfd390df3
%       2) Run this file.

close all
clear class
clear all %#ok
clc

% use the namespace
include_namespace_dq

%% V-REP setup
% Create a DQ_VrepInterface object and start communication with V-REP
vi = DQ_VrepInterface();
vi.connect('127.0.0.1',19997);
disp('Communication established!')

% Start simulation
vi.set_synchronous(true);
vi.start_simulation();
disp('Simulation started!')

% Define robot interface
robot_name = 'LBR4p';
for i=1:7
    current_joint_name = {robot_name,'_joint',int2str(i)};
    joint_names{i} = strjoin(current_joint_name,''); %#ok
end

%% Definition of the actuation torque
tau = [-0.005; -25; 0; -30; -0.007; 0; 0];

%% General variables
iteration = 0;
max_iteration = 10;

%% Main loop
while(iteration <= max_iteration)
    % Robot actuation in V-REP
    vi.set_joint_torques(joint_names, tau);
    vi.trigger_next_simulation_step();
    vi.wait_for_simulation_step_to_end();

    formatSpec = 'Actuation torque sent to V-REP: %f\n';
    fprintf(formatSpec,tau);
    disp(' ')

    % Read joint torques from V-REP
    tau_read = vi.get_joint_torques(joint_names);

    formatSpec = 'Joint torque read from V-REP: %f\n';
    fprintf(formatSpec,tau_read);
    disp(' ')

    %% General messages
    iteration = iteration + 1;

    formatSpec = 'Iteration: %f\n';
    fprintf(formatSpec,iteration);
    disp(' ')
end

%% Finishes V-REP communication
vi.stop_simulation();
disp('Simulation finished!')
vi.disconnect_all();
disp('Communication finished!')