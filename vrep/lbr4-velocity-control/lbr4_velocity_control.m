%% Example of velocity actuation and joint position reading
% Frederico Fernandes Afonso Silva - Jan/2024
% Last modification: Jan/2024

% Example of velocity control and reading using a KUKA LBR4 robot.
%
% Usage:
%       1) Open scene "lbr4_velocity_control.ttt" on V-REP.
%           Available at: https://osf.io/gzrby
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

% Finish any previous V-REP communication
vi.stop_simulation();
vi.disconnect_all();

% Start a new connection
vi.connect('127.0.0.1',19997);
disp('Communication established!')

% Start simulation
vi.set_synchronous(true);
vi.start_simulation();
disp('Simulation started!')

% Define robot interface
robot_vrep = LBR4pVrepRobot('LBR4p', vi);

%% Definition of the actuation velocity
qd = [-pi/4; -5*pi; 0; pi/2; pi/2; pi/2; pi];

%% General variables
iteration = 0;
max_iteration = 10;

%% Main control loop for the first desired pose
while(iteration <= max_iteration)
    % Robot actuation in V-REP
    robot_vrep.set_target_configuration_space_velocities(qd);
    vi.trigger_next_simulation_step();
    vi.wait_for_simulation_step_to_end();

    formatSpec = 'Joint velocity sent to V-REP: %f\n';
    fprintf(formatSpec,qd);
    disp(' ')

    % Read joint positions from V-REP
    q_read = robot_vrep.get_configuration_space_positions;

    formatSpec = 'Joint position read from V-REP: %f\n';
    fprintf(formatSpec,q_read);
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