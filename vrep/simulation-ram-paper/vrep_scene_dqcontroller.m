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

function vrep_scene_dqcontroller(simulation_parameters)

include_namespace_dq;

%% Simulation convenience flags
SHOW_FRAMES = simulation_parameters.show_frames;

%% Initialize V-REP interface
vi = DQ_VrepInterface;
vi.disconnect_all(); % For testing, can be removed for release
vi.connect('127.0.0.1',19997);
vi.start_simulation();

%% Initialize VREP Robots
lwr4_vreprobot = LBR4pVrepRobot('LBR4p',vi);
youbot_vreprobot = YouBotVrepRobot('youBot',vi);

%% Get robot joint information from V-REP
youbot_reference_frames = {'YoubotArm0','YoubotArm1','YoubotArm2','YoubotArm3','YoubotArm4'};

%% Load DQ Robotics kinematics
lwr4  = lwr4_vreprobot.kinematics();
youbot = youbot_vreprobot.kinematics();

%% Initialize controllers
lwr4_controller = DQ_PseudoinverseController(lwr4);
lwr4_controller.set_control_objective(ControlObjective.Pose);
lwr4_controller.set_gain(10);

solver = DQ_QuadprogSolver;
youbot_controller = DQ_ClassicQPController(youbot,solver);
youbot_controller.set_control_objective(ControlObjective.Pose);
youbot_controller.set_gain(10);
youbot_controller.set_damping(0.01);

sampling_time = 0.05; % V-REP's sampling time is 50 ms.
total_time = simulation_parameters.total_time; % Total time, in seconds.

%% Set the initial robots configurations
lwr4_q  = [0; 1.7453e-01; 0; 1.5708e+00; 0; 2.6273e-01; 0];
lwr4_vreprobot.send_q_to_vrep(lwr4_q);
% Get current end-effector pose with respect to its own base
lwr4_x0  = lwr4.get_reference_frame()'*lwr4.fkm(lwr4_q);
youbot_q = youbot_vreprobot.get_q_from_vrep();

%% Initalize arrays to store all signals resulting from the simulation
max_iterations = round(total_time/sampling_time);
youbot_tracking_error_norm = zeros(1,max_iterations);
youbot_control_inputs = zeros(youbot.get_dim_configuration_space(),...
    max_iterations);
youbot_q_vector = zeros(youbot.get_dim_configuration_space(), ...
    max_iterations);

lwr4_tracking_error_norm = zeros(1,max_iterations);
lwr4_control_inputs = zeros(lwr4.get_dim_configuration_space(),...
    max_iterations);
lwr4_q_vector = zeros(lwr4.get_dim_configuration_space(), ...
    max_iterations);

% index of the auxiliary arrays used to plot data
ii = 1;

%% Control loop
for t=0:sampling_time:total_time
    
    %% Get obstacles from V-REP
    plane = get_plane_from_vrep(vi,'ObstaclePlane',DQ.k);
    cylinder1 = get_line_from_vrep(vi,'ObstacleCylinder1',DQ.k);
    cylinder2 = get_line_from_vrep(vi,'ObstacleCylinder2',DQ.k);
    
    %% Set reference for the manipulator and the mobile manipulator
    [lwr4_xd, lwr4_ff] = compute_lwr4_reference(lwr4, simulation_parameters, ...
        lwr4_x0,t);
    
    [youbot_xd, youbot_ff] = compute_youbot_reference(youbot_controller, ...
        lwr4_xd, lwr4_ff);
    
   
    %% Compute control signal for the arm
    lwr4_u  = lwr4_controller.compute_tracking_control_signal(lwr4_q,...
        vec8(lwr4_xd),vec8(lwr4_ff));
    
    
    %% Compute control signal for the youbot
    % first calculate the constraints
    [Jconstraint, bconstraint] = compute_constraints(youbot, youbot_q, ...
        plane,cylinder1,cylinder2);
    youbot_controller.set_inequality_constraint(-Jconstraint,1*bconstraint);
    
    % compute control signal
    youbot_u = youbot_controller.compute_tracking_control_signal(youbot_q,...
        vec8(youbot_xd), vec8(youbot_ff));
    
    % since we are using V-REP just for visualization, integrate the
    % control signal to update the robots configurations
    lwr4_q = lwr4_q + sampling_time*lwr4_u;
    youbot_q = youbot_q + sampling_time*youbot_u;
    
    %% Send desired values
    lwr4_vreprobot.send_q_to_vrep(lwr4_q);
    youbot_vreprobot.send_q_to_vrep(youbot_q);
    
    %% Show frames, for testing. This if (and the following else)
    % can be removed for release
    if SHOW_FRAMES
        % Plot current LBR4p end-effector pose on V-REP
        vi.set_object_pose('x1', lwr4.fkm(lwr4_q));
        % Plot desired LBR4p end-effector pose on V-REP
        vi.set_object_pose('xd1', lwr4_xd);
        % Plot current youbot end-effector pose on V-REP
        vi.set_object_pose('x2', youbot.fkm(youbot_q));
        % plot desired youbot end-effector pose on V-REP
        vi.set_object_pose('xd2', youbot_xd);
        % Show youbot's base frame in V-REP
        vi.set_object_pose('YoubotKinematicBase', ...
            youbot.fkm(youbot_q,1))
        % Show youbot's arm frames on V-REP
        for k=1:5
            vi.set_object_pose(youbot_reference_frames{k}, ...
                youbot.fkm(youbot_q,2,k))
        end
    else
        vi.set_object_pose('x1',DQ(1));
        vi.set_object_pose('xd1',DQ(1));
        vi.set_object_pose('x2',DQ(1));
        vi.set_object_pose('xd2',DQ(1));
        vi.set_object_pose('YoubotKinematicBase',DQ(1));
        for k=1:5
            vi.set_object_pose(youbot_reference_frames{k},DQ(1));
        end
    end
    
    %% Get data to plot them later
    youbot_tracking_error_norm(:,ii) = norm(youbot_controller.get_last_error_signal());
    youbot_control_inputs(:,ii) = youbot_u;
    youbot_q_vector(:,ii) = youbot_q;
    lwr4_tracking_error_norm(:,ii) = norm(lwr4_controller.get_last_error_signal());
    lwr4_control_inputs(:,ii) = lwr4_u;
    lwr4_q_vector(:,ii) = lwr4_q;
    ii = ii + 1;
end


%% End V-REP
vi.stop_simulation();
vi.disconnect();

end

function line = get_line_from_vrep(vrep_interface,object_name,primitive)
line_object_pose = vrep_interface.get_object_pose(object_name);
p = translation(line_object_pose);
r = rotation(line_object_pose);
l = Ad(r,primitive);
m = cross(p,l);
line = l + DQ.E*m;
end

function plane = get_plane_from_vrep(vrep_interface,object_name,primitive)
plane_object_pose = vrep_interface.get_object_pose(object_name);
p = translation(plane_object_pose);
r = rotation(plane_object_pose);
n = Ad(r,primitive);
d = dot(p,n);
plane = n + DQ.E*d;
end

% compute the time-varying reference for the Kuka LWR 4 end-effector
function [x, xdot] = compute_lwr4_reference(lwr4, simulation_parameters, x0, t)
include_namespace_dq;

% parameters for the trajectory
dispz = simulation_parameters.dispz;
wd = simulation_parameters.wd;
wn = simulation_parameters.wn;

% calculate the time-varing plane orientation with respect to the robot
% BASE.
phi = (pi/2)*sin(wn*t);
r = cos(phi/2) + k_ * sin(phi/2);

z = dispz*cos(wd*t)*k_;
p = 1 + E_ * 0.5 * z;

x = r*x0*p;

% calculate its time derivative
phidot = (pi/2)*cos(wn*t)*wn;
rdot = 0.5*(-sin(phi/2) + k_*cos(phi/2))*phidot;
pdot = -E_*0.5*dispz*wd*sin(wd*t)*k_;

xdot = rdot*x0*p + r*x0*pdot;

% The trajectory,including the feedforward term, has been calculated with
% respect to the manipulator base. Therefore, we need to calculate them
% with respect to the global reference frame.
x = lwr4.get_reference_frame()*x;
xdot = lwr4.get_reference_frame()*xdot;
end

function [youbot_xd, youbot_ff] = compute_youbot_reference(controller, ...
    lbr4p_xd, lbr4p_ff)    
    persistent tc;
    persistent rcircle;
    persistent first_iteration;
    
    include_namespace_dq;

    circle_radius = 0.1;
    tcircle = 1 + E_ * 0.5 * circle_radius * j_;
    
    %% Youbot trajectory
    % Those are the trajectory components to track the whiteboard
    youbot_xd = lbr4p_xd * (1 + 0.5*E_*0.015*k_) * j_;
    youbot_ff = lbr4p_ff * (1 + 0.5*E_*0.015*k_) * j_;
    % Now we modify the trajectory in order to draw a circle, but we do it
    % only if the whiteboard pen tip is on the whiteboard surface.
    if isempty(first_iteration)
        first_iteration = false;
        tc = 0;
        rcircle = DQ(1);        
    elseif norm(controller.get_last_error_signal()) < 0.002
        tc = tc + 0.1; % Increment around 0.5 deg.
        rcircle = cos(tc/2) + k_ * sin(tc/2);
    end
    youbot_xd = youbot_xd * rcircle * tcircle;
    youbot_ff = youbot_ff * rcircle * tcircle;
end
    

function [Jconstraint, bconstraint] = compute_constraints(youbot, youbot_q, plane, cylinder1, cylinder2)
% We define differential inequalities given by d_dot >= -eta*d,
% where d is the distance from the end-effector to the
% primitive, d_dot is its time derivative and eta determines the
% maximum rate for the approach velocity.

% This information should be inside the robot itself
robot_radius = 0.35;
radius_cylinder1 = 0.1;
radius_cylinder2 = 0.1;

include_namespace_dq

% Get base translation and translation Jacobian
youbot_base = youbot.get_chain(1);
youbot_base_pose = youbot_base.raw_fkm(youbot_q);
Jx = youbot_base.raw_pose_jacobian(youbot_q);
t = translation(youbot_base_pose);
Jt = [youbot.translation_jacobian(Jx,youbot_base_pose),zeros(4,5)];

% First we calculate the primitives for the plane
Jdist_plane = youbot.point_to_plane_distance_jacobian(Jt, t, plane);
dist_plane = DQ_Geometry.point_to_plane_distance(t,plane) - robot_radius;

% Now we compute the primitives for the two cylinders
Jdist_cylinder1 = youbot.point_to_line_distance_jacobian(Jt, t, cylinder1);
dist_cylinder1 = DQ_Geometry.point_to_line_squared_distance(t,cylinder1) - ...
    (radius_cylinder1 + robot_radius)^2;

Jdist_cylinder2 = youbot.point_to_line_distance_jacobian(Jt, t, cylinder2);
dist_cylinder2 = DQ_Geometry.point_to_line_squared_distance(t,cylinder2) - ...
    (radius_cylinder2 + robot_radius)^2;

% Assemble all constraint matrices into a unique matrix
Jconstraint = [Jdist_plane; Jdist_cylinder1; Jdist_cylinder2];
% And do analagously to the constraint vectors
bconstraint = [dist_plane; dist_cylinder1; dist_cylinder2];
end