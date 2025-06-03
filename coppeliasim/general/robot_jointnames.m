% This example shows how to use basic commands in CoppeliaSim.
%
% Note:
%
% Open the DQ_Robotics_lab.ttt scene (https://github.com/dqrobotics/coppeliasim-scenes) 
% in CoppeliaSim before running this script.
% 
% (C) Copyright 2011-2025 DQ Robotics Developers
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
% DQ Robotics website: https://dqrobotics.github.io/
%
% Contributors to this file:
%    1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)





clear;
close all;
clc;

include_namespace_dq;

% Instantiate the class
cs = DQ_CoppeliaSimInterfaceZMQ();


try
    % Establish the connection with CoppeliaSim. 
    cs.connect();

    % Starts the simulation. 
    cs.start_simulation();

    robotnames = {"UR5", "Franka", "UMIRobot"};

    for i=1:length(robotnames)
        disp(robotnames{i} +" robot configuration");
        cs.get_joint_positions(cs.get_jointnames_from_object(robotnames{i}))'
    end

    cs.stop_simulation(); % Stop the simulation

catch ME
    cs.stop_simulation(); % Stop the simulation
    rethrow(ME)
end

