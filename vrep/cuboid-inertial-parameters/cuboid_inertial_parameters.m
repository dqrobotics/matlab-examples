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

%% Get the Cuboid's inertial parameters with respect to its shape form
disp('Cuboid`s inertial parameters with respect to its shape form:')

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

%% Get the Cuboid's inertial parameters with respect to ReferenceFrame
disp('--------------------------------------------------------------')
disp(' ')

disp('Cuboid`s inertial parameters with respect to ReferenceFrame:')

% Get the ReferenceFrame's handle
handle_ref = vi.get_handle('ReferenceFrame');

% Get the Cuboid's inertia matrix
inertia_matrix_in_rf = vi.get_inertia_matrix(handle, handle_ref);
disp('inertia matrix =')
disp(inertia_matrix_in_rf)

% Get the Cuboid's center of mass
center_of_mass_in_rf = vi.get_center_of_mass(handle, handle_ref);
disp('center of mass =')
disp(center_of_mass_in_rf)
disp(' ')

%% Finishes V-REP communication
vi.stop_simulation();
disp('Simulation finished!')
vi.disconnect_all();
disp('Communication finished!')