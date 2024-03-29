% panda_kinematic_control() performs a pseudoinverse pose control using a
% Franka Emika Panda robot.

% (C) Copyright 2011-2023 DQ Robotics Developers
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
%     Frederico Fernandes Afonso Silva - frederico.silva@ieee.org

function panda_kinematic_control()
    % use the namespace
    include_namespace_dq

    % Create a new DQ_kinematics object with the Franka Emika Panda arm
    % modified Denavit-Hartenberg parameters
    panda = FrankaEmikaPandaRobot.kinematics();

    % The controller is given by
    % u = -pinv(J)*gain*task_error
    % where J is the robot Jacobian, gain determines the convergence rate,
    % and task_error is the error between the current task variable and
    % the desired one. When the task error derivative is below
    % stability_threshold, the closed-loop system is said to have reached a
    % stable region.
    controller = DQ_PseudoinverseController(panda);
    controller.set_gain(100);
    controller.set_stability_threshold(0.0001);

    % Initial configuration
    q = [0; 0; 0; -pi/2; 0; pi/2; 0];

    % Integration step
    T = 0.001;

    % The task variable to be controlled is the end-effector pose
    controller.set_control_objective(ControlObjective.Pose);

    % Generate a desired end-effector pose
    xd = 1 + 0.5*E_*(i_*0.5 + j_*0.3 + k_*0.4);
    task_reference = vec8(xd);

    % Prepare the visualization
    figure;
    view(70,25);
    axis equal;
    axis([-0.1, 0.6,-0.4, 0.6, -0.1, 0.8])
    hold on;
    plot(panda, q, 'nojoints');
    plot(xd, 'scale', 0.1);    

    % This is actually the important part on how to use the controller
    while ~controller.system_reached_stable_region()
        % Let us calculate the control input
        u = controller.compute_setpoint_control_signal(q, task_reference);

        % Do a numerical integration to update the robot in Matlab. In an
        % actual robot actuated by means of velocity inputs, this step is
        % not necessary
        q = q + T*u;

        % Draw the robot in Matlab
        plot(panda, q', 'nojoints');
        
        pvec = vec3(panda.fkm(q).translation);
        plot3(pvec(1),pvec(2),pvec(3), 'ro');
        drawnow;
    end
end
