%% Example of torque control and force sensor reading
% Frederico Fernandes Afonso Silva - Jul/2023
% Last modification: Jul/2023

% Example of torque control and force sensor reading using a Franka Emika
% Panda robot.
%
% Usage:
%       1) Open scene "panda_torque_control.ttt" on V-REP.
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
vi.trigger_next_simulation_step(); % force sensor returns noise values if called before the first trigger
vi.wait_for_simulation_step_to_end();

% Define robot interface
robot_vrep = FrankaEmikaPandaVrepRobot('Franka', vi);

force_sensor_name = '/Franka/connection';

%% Definition of the actuation torque
tau = [-0.005; -70; 0; 5; -0.007; 0; 0];

%% General variables
iteration = 1;
max_iteration = 10;

%% Main control loop for the first desired pose
while(iteration <= max_iteration)
    % Robot actuation in V-REP
    robot_vrep.send_tau_to_vrep(tau);
    vi.trigger_next_simulation_step();
    vi.wait_for_simulation_step_to_end();

    formatSpec = 'Actuation torque sent to V-REP: %f\n';
    fprintf(formatSpec,tau);
    disp(' ')

    % Read joint torques from V-REP
    tau_read = robot_vrep.get_tau_from_vrep;

    formatSpec = 'Joint torque read from V-REP: %f\n';
    fprintf(formatSpec,tau_read);
    disp(' ')

    % Get force sensor readings 
    [force_vec, torque_vec] = vi.get_force_sensor_readings(force_sensor_name);
    wrench_sensor = DQ([force_vec torque_vec]);

    formatSpec = 'Wrench read from V-REP (at sensor`s reference frame): %f\n';
    fprintf(formatSpec,vec6(wrench_sensor));
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