%% Example of geting the inertial parameters of a cuboid on CoppeliaSim
% Frederico Fernandes Afonso Silva - Jan/2024
% Last modification: Feb/2024

% Example of geting the inertial parameters of a cuboid on CoppeliaSim.
%
% Usage:
%       1) Open scene "cuboid_inertial_parameters.ttt" on V-REP.
%           Available at: https://osf.io/jeut7
%       2) Run this file.

close all
clear class
clear all %#ok
clc

%% V-REP setup
% Create a DQ_VrepInterface object and start communication with V-REP
vi = DQ_VrepInterface();

% Finish any previous V-REP communication
vi.stop_simulation();
vi.disconnect_all();

% Start a new connection
vi.connect('127.0.0.1',19997);
disp('Communication established!')

% Start simulation otherwise child scripts cannot be called
vi.set_synchronous(true);
vi.start_simulation();
disp('Simulation started!')

vi.trigger_next_simulation_step();
vi.wait_for_simulation_step_to_end();

%% Get the Cuboid's inertial parameters with respect to its shape frame
disp('Cuboid`s inertial parameters with respect to its shape frame:')
disp(' ')

% Get the Cuboid's handle
handle = vi.get_handle('Cuboid');

% Get the Cuboid's mass
mass_in_sf = vi.get_mass(handle);
formatSpec = 'mass = %f\n';
fprintf(formatSpec, mass_in_sf);
disp(' ')

% Get the Cuboid's inertia matrix
inertia_matrix_in_sf = vi.get_inertia_matrix(handle);
disp('inertia matrix =')
disp(inertia_matrix_in_sf)

% Get the Cuboid's center of mass
center_of_mass_in_sf = vi.get_center_of_mass(handle);
disp('center of mass =')
disp(center_of_mass_in_sf)
disp(' ')

%% Get the Cuboid's inertial parameters with respect to the absolute reference frame
disp('--------------------------------------------------------------')
disp(' ')

% Get the absolute reference frame's name
ref_name = vi.ABSOLUTE_FRAME;

formatSpec = 'Cuboid`s inertial parameters with respect to %s:\n';
fprintf(formatSpec, ref_name);

% Get the Cuboid's inertia matrix
inertia_matrix_in_rf = vi.get_inertia_matrix(handle, ref_name);
disp('inertia matrix =')
disp(inertia_matrix_in_rf)

% Get the Cuboid's center of mass
center_of_mass_in_rf = vi.get_center_of_mass(handle, ref_name);
disp('center of mass =')
disp(center_of_mass_in_rf)
disp(' ')

%% Finishes V-REP communication
vi.stop_simulation();
disp('Simulation finished!')
vi.disconnect_all();
disp('Communication finished!')