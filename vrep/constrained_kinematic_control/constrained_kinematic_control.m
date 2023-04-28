% (C) Copyright 2023 DQ Robotics Developers
% This file is part of DQ Robotics.
%
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published 
%     by the Free Software Foundation, either version 3 of the License, or
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
%     Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)

clear all;
close all;
clc;

include_namespace_dq

vi = DQ_VrepInterface();
vi.disconnect_all();
vi.connect('127.0.0.1',19997);
vi.set_synchronous(true);
vi.start_simulation();

try

jointnames={'LBR4p_joint1','LBR4p_joint2',...
            'LBR4p_joint3','LBR4p_joint4',...
            'LBR4p_joint5','LBR4p_joint6',...
            'LBR4p_joint7'};

    %---------------------Robot definition--------------------------%
    LBR4p_DH_theta = [0, 0, 0, 0, 0, 0, 0];
    LBR4p_DH_d = [0.200, 0, 0.4, 0, 0.39, 0, 0];
    LBR4p_DH_a = [0, 0, 0, 0, 0, 0, 0];
    LBR4p_DH_alpha = [pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0];
    LBR4p_DH_type = double(repmat(DQ_JointType.REVOLUTE,1,7));
    LBR4p_DH_matrix = [LBR4p_DH_theta;
                        LBR4p_DH_d;
                        LBR4p_DH_a;
                        LBR4p_DH_alpha
                        LBR4p_DH_type];
                
    robot = DQ_SerialManipulatorDH(LBR4p_DH_matrix);
    xbase = vi.get_object_pose('LBR4p_joint1');
    robot.set_reference_frame(xbase);
    robot.set_base_frame(xbase);
    robot.set_effector(1+0.5*DQ.E*DQ.k*0.07);
    xbase = robot.get_base_frame();
    n = robot.get_dim_configuration_space();
    qmin = [-pi -pi -pi/2 -pi -pi/2 -pi -pi/2]';
    qmax = [pi pi pi/2 pi pi/2 pi pi/2]';
    %----------------------------------------------------------------
    %---------------------Controller definition----------------------%
    qp_solver = DQ_QuadprogSolver();
    controller = DQ_ClassicQPController(robot, qp_solver);
    controller.set_gain(0.2);
    controller.set_damping(0.05);
    controller.set_control_objective(ControlObjective.DistanceToPlane)
    
    xplane = vi.get_object_pose('target_plane');
    plane_d = Adsharp(xplane,-k_);
    controller.set_target_primitive(plane_d);
    controller.set_stability_threshold(0.00001);
    %----------------------------------------------------------------
    %  Plane constraints
    safe_distance = 0.1;
    wall_plane  = Adsharp(vi.get_object_pose('wall_plane'),  k_); 
    floor_plane = Adsharp(vi.get_object_pose('floor_plane'), k_);
    back_plane  = Adsharp(vi.get_object_pose('back_plane'),  k_);
    planes = {wall_plane, floor_plane, back_plane};

    vi.trigger_next_simulation_step();
    q =  vi.get_joint_positions(jointnames);
    %----------------------------------------------------------------
    j=1;
    while ~controller.system_reached_stable_region()
        %----------VFIs------------------------------------
        q =  vi.get_joint_positions(jointnames);
        x_pose = robot.fkm(q);
        p = translation(x_pose); 
        J_pose = robot.pose_jacobian(q); 
        J_trans = robot.translation_jacobian(J_pose,x_pose);
        
        for i=1:size(planes,2)
            Jdists(i,:) = robot.point_to_plane_distance_jacobian(J_trans, p, planes{i});
            dists(i)    = DQ_Geometry.point_to_plane_distance(p, planes{i}) -safe_distance;
        end

        A = [-Jdists; -eye(n); eye(n)];
        b = [dists'; (q-qmin); -(q-qmax)];
        controller.set_inequality_constraint(A,b);
        %-------------------------------------------------
        u = controller.compute_setpoint_control_signal(q, 0);
        vi.set_joint_target_velocities(jointnames, u);
        vi.set_object_pose('effector', robot.fkm(q))
        vi.set_object_pose('base', xbase);
        norm(controller.get_last_error_signal())
        vi.trigger_next_simulation_step();
        %----------------
        u_log(:,j) = u;
        q_dot(:,j) = vi.get_joint_velocities(jointnames);
        j = j+1;

    end
    show_joint_velocities(u_log, q_dot);
    vi.stop_simulation();
    vi.disconnect();

catch ME
    vi.stop_simulation();
    vi.disconnect();
    rethrow(ME)
end