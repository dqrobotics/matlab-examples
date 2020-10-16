% PLOT_FREE_FLYING_ROBOT() runs a simple example to test the DQ_FreeFlyingRobot 
% class. 
%

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

function plot_free_flying_robot()
    include_namespace_dq;
    
    robot1 = DQ_FreeFlyingRobot();  
    robot1.set_radius(0.1);
    
    robot2 = DQ_FreeFlyingRobot();  
    robot2.set_radius(0.5);
    
    robot3 = DQ_FreeFlyingRobot();  
    robot3.set_radius(1);
    
    p = i_;
    
    plot(DQ(1));
    hold on;
    axis equal;
    
     for phi = 0:0.01:2*pi
        r1 = cos(4*phi/2) + k_ * sin(4*phi/2);
        r2 = cos(2*phi/2) + k_ * sin(2*phi/2);
        r3 = cos(phi/2) + k_ * sin(phi/2);
        
        x1 = r1*(1+E_*p/2);
        x2 = r2*(1+E_*p*5/2);
        x3 = r3*(1+E_*p*10/2);
        
        plot(robot1,x1);
        plot(robot2,x2);
        plot(robot3,x3);
        pause(0.1)
     end
end