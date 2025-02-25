% This example shows how to use the stepping mode.
% The example performs a free fall test of an object and shows the 
% estimated and measured height of the object after some seconds.
% 
% (C) Copyright 2025 DQ Robotics Developers
% 
% This file is part of DQ Robotics.
% 
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%    1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)

clear;
close all;
clc;


% Instantiate the class
cs = DQ_CoppeliaSimInterfaceZMQ();


try
    % Establish the connection with CoppeliaSim. If the simulation is running in the
    % same machine of this script, you can use "localhost". Furthermore, we
    % use the default port 23000.
    cs.connect("localhost", 23000);

    % Set the stepping mode. 
    % Check more in https://manual.coppeliarobotics.com/en/simulation.htm
    cs.set_stepping_mode(true);

    % Starts the simulation. 
    cs.start_simulation();

    % Read the position [x y z] of the Sphere object in CoppeliaSim
    p0 = vec3(cs.get_object_translation('/Sphere'));
    y0 = p0(3); % Height of the object.

    % Define the simulation time step to match the simulation dt in
    % CoppeliaSim. 
    % Check more in https://manual.coppeliarobotics.com/en/simulationPropertiesDialog.htm#:~:text=Simulation%20dt%3A%20the%20simulation%20time,dynamics%20dt%20(see%20below).
    time_simulation_step = 0.05;
    gravity = -9.81;
    disp('---------------------------------')
    disp(['Initial height: ',num2str(y0)])
    disp('---------------------------------')

    for i=0:4
        t = i*time_simulation_step;
        p = vec3(translation(cs.get_object_pose('/Sphere')));
        y_sim = p(3);
        y_est = y0 - 0.5*gravity*t^2; 

        % Trigger a simulation step in CoppeliaSim
        cs.trigger_next_simulation_step();
    end
    disp(['Elapsed time: ',num2str(t)])
    disp(['Estimated height: ',num2str(y_est),'. Measured height: ', num2str(y_sim)])
    cs.stop_simulation(); % Stop the simulation

catch ME

    cs.stop_simulation(); % Stop the simulation
    rethrow(ME)

end