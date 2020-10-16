% CONTROL_FREE_FLYING_ROBOT() runs a simple example to control a
% free-flying robot using a control law based on the logarithmic mapping.
%
% The control law is inspired on the techniques presented in  H. J.
% Savino, L. C. A. Pimenta, J. A. Shah, and B. V. Adorno, “Pose consensus
% based on dual quaternion algebra with application to decentralized
% formation control of mobile manipulators,” Journal of the Franklin
% Institute, vol. 357, no. 1, pp. 142–178, Jan. 2020.
% 
% The idea is just to present the functionalities of DQ_FreeFlyingRobot
% methods.


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

function control_free_flying_robot()
    include_namespace_dq;
    
    robot = DQ_FreeFlyingRobot(); 
    robot.set_radius(0.5);
    
    T = 0.01;
    lambda = 1;    
    
    xd = (1 + 0.5*E_*10*DQ(rand(3,1)))*normalize(DQ(rand(4,1)));
    x = normalize(DQ(rand(8,1)));
    
    plot(DQ(1), 'name', 'Inertial frame');
    hold on;
    plot(xd, 'name','Desired pose');
    axis equal;
    
    
    xerror = x'*xd;
    
    while norm(vec6(log(xerror))) > 0.001
        % Compute the robot pose Jacobian that satisfies vec8(xdot) =
        % J*vec8(twist)
        J = robot.pose_jacobian(x);
        
        %compute the control input u
        % Since we just want to test the  DQ_FreeFlyingRobot methods, we
        % are using pinv(J). However J is invertible and its inverse is
        % given by haminus8(2*x').
        uvec = pinv(J)*(-lambda*DQ.C8*haminus8(xd')*Q8(xerror)*vec6(log(xerror)));
        
        u = DQ(uvec);
        % Now we update the robot pose by integrating the control input u:
        x = exp(T/2*u)*x;
        
        xerror = x'*xd;
        
        plot(robot,x);
        drawnow;
    end
end