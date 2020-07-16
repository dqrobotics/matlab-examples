% Example used in the paper Adorno and Marinho. 2020. “DQ Robotics: A
% Library for Robot Modeling and Control.” IEEE Robotics & Automation
% Magazine, https://doi.org/10.1109/MRA.2020.2997920.
%
% Usage: Assuming that V-REP is already working with Matlab,
%        1) open the file vrep_scene_felt_pen_official_scene.ttt on V-REP;
%        2) run this file;
%        3) adjust the parameters below if you want to change the
%        trajectory parameters, simulation time, etc.

% (C) Copyright 2020 DQ Robotics Developers
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
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ieee.org

clear all;
close all;
clc;


% False if we don' want to show intermediate frames
simulation_parameters.show_frames = false;
% Total simulation time, in seconds.
simulation_parameters.total_time = 200;

% The maximum radius that the manipulator will perform
simulation_parameters.dispz = 0.1;
% Occilation along the radius
simulation_parameters.wd = 0.5;
% Occilation around the rotation axix
simulation_parameters.wn = 0.1;

vrep_scene_dqcontroller(simulation_parameters);


