% (C) Copyright 2020 DQ Robotics Developers
%
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
%     Juan Jose Quiroz Omana - juanjqo@g.ecc.u-tokyo.ac.jp

function stanford_manipulator_modeling()
% This example shows how to declare the Stanford Manipulator using both DH convention
% (Standard and Modified). The former uses DQ_SerialManipulatorDH and the latter uses
% DQ_SerialManipulatorMDH. In addition, this example performs a comparison
% between them.

        %% Robot Declaration using the Standard DH Convention 
        % (See Table 3.4 from Robot Modeling and Control Second Edition, Spong, Mark W.
        %  Hutchinson, Seth M., Vidyasagar)

        d2 = 0.4;
        d3 = 0.1;
        d6 = 0.3;
        robot_DH_theta=[0,0,0,0,0,0];
        robot_DH_d = [0,d2,d3,0,0,d6];
        robot_DH_a = [0, 0,0,0,0,0];
        robot_DH_alpha = [-pi/2, pi/2, 0,-pi/2,pi/2,0];        
        R = DQ_SerialManipulatorDH.JOINT_ROTATIONAL;
        P = DQ_SerialManipulatorDH.JOINT_PRISMATIC;
        robot_type = [R,R,P,R,R,R];
        robot_DH_matrix = [robot_DH_theta;
                           robot_DH_d;
                           robot_DH_a;
                           robot_DH_alpha;
                           robot_type];
        StanfordDHRobot = DQ_SerialManipulatorDH(robot_DH_matrix);


        %% Robot Declaration using the Modified DH Convention 
        % (See Table 2.1 from Foundations of Robotics, Tsuneo Yoshikawa)
        robot_MDH_theta=[0,0,0,0,0,0];
        robot_MDH_d = [0,d2,d3,0,0,0];
        robot_MDH_a = [0, 0,0,0,0,0];
        robot_MDH_alpha = [0,-pi/2, pi/2, 0,-pi/2,pi/2];  
        robot_MDH_matrix = [robot_MDH_theta;
                            robot_MDH_d;
                            robot_MDH_a;
                            robot_MDH_alpha;
                            robot_type];
        StanfordMDHRobot = DQ_SerialManipulatorMDH(robot_MDH_matrix);
        StanfordMDHRobot.set_effector(1+DQ.E*0.5*DQ.k*d6);

        
        number_of_trials = 100;  

        % Initial robot configuration
        q_list = rand(6,number_of_trials);
         % Initial robot velocity configuration
        q_dot_list = rand(6,number_of_trials);

        % Testing some methods
        TestCase = matlab.unittest.TestCase.forInteractiveUse;

%       The following tests will be addressed in futures versions.
%
%         TestCase.assertEqual(StanfordDHRobot.get_thetas(),robot_DH_theta, "AbsTol", DQ.threshold,...
%                 "Error in DQ_SerialManipulatorDH.get_thetas()");
% 
%         TestCase.assertEqual(StanfordDHRobot.get_ds(),robot_DH_d, "AbsTol", DQ.threshold,...
%         "Error in DQ_SerialManipulatorDH.get_ds()");
% 
%         TestCase.assertEqual(StanfordDHRobot.get_as(),robot_DH_a, "AbsTol", DQ.threshold,...
%         "Error in DQ_SerialManipulatorDH.get_as()");
%         
%         TestCase.assertEqual(StanfordDHRobot.get_alphas(),robot_DH_alpha, "AbsTol", DQ.threshold,...
%         "Error in DQ_SerialManipulatorDH.get_alphas()");
% 
%         TestCase.assertEqual(StanfordDHRobot.get_types(),robot_type, "AbsTol", DQ.threshold,...
%         "Error in DQ_SerialManipulatorDH.get_types()");
% 
% 
% 
%         TestCase.assertEqual(StanfordMDHRobot.get_thetas(),robot_MDH_theta, "AbsTol", DQ.threshold,...
%                 "Error in DQ_SerialManipulatorMDH.get_thetas()");
% 
%         TestCase.assertEqual(StanfordMDHRobot.get_ds(),robot_MDH_d, "AbsTol", DQ.threshold,...
%         "Error in DQ_SerialManipulatorMDH.get_ds()");
% 
%         TestCase.assertEqual(StanfordMDHRobot.get_as(),robot_MDH_a, "AbsTol", DQ.threshold,...
%         "Error in DQ_SerialManipulatorMDH.get_as()");
%         
%         TestCase.assertEqual(StanfordMDHRobot.get_alphas(),robot_MDH_alpha, "AbsTol", DQ.threshold,...
%         "Error in DQ_SerialManipulatorMDH.get_alphas()");
% 
%         TestCase.assertEqual(StanfordMDHRobot.get_types(),robot_type, "AbsTol", DQ.threshold,...
%         "Error in DQ_SerialManipulatorMDH.get_types()");



        
        for i=1:number_of_trials        
            q = q_list(:,i);
            q_dot = q_dot_list(:,i);
            x1 = StanfordDHRobot.fkm(q);
            J1 = StanfordDHRobot.pose_jacobian(q);
            J1_dot = StanfordDHRobot.pose_jacobian_derivative(q,q_dot);  

            x2 = StanfordMDHRobot.fkm(q);
            J2 = StanfordMDHRobot.pose_jacobian(q);
            J2_dot = StanfordMDHRobot.pose_jacobian_derivative(q,q_dot);
            
            TestCase.assertEqual(vec8(x1),vec8(x2), "AbsTol", DQ.threshold,...
                "Error in DQ_SerialManipulatorMDH.fkm");
            TestCase.assertEqual(J1,J2, "AbsTol", DQ.threshold,...
                "Error in DQ_SerialManipulatorMDH.pose_jacobian");
            TestCase.assertEqual(J1_dot,J2_dot,"AbsTol", DQ.threshold,...
                "Error in DQ_SerialManipulatorMDH.pose_jacobian_derivative");

        end

         




end

