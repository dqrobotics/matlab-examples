function example_damped_numerical_filtered_controller()
    format SHORTE;

    %% Chiaverini's Controller Implementation
    % Uses variable damping (lambda) with numerical filtering along with a
    % isotropic damping (beta).

    %% Constant Initialization.
    % Convergence Tolerance
    convergence_tolerance = 1.e-5;
    % Constants
    pi2 = pi/2;
    % Controller Gains And Dampings
    lambda_max  = 0.01;  %Variable Damping
    beta        = 0.01;  %Isotropic Damping
    epsilon     = 0.001; %Singular region size
    kp = diag([0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8]); %Proportional Gain
    % Robot Initial Positions
    q = [0;pi2;0;0;0;0;0];
    % Desired End Effector Final Pose
    xd = DQ([1,0,0,0,0,0,0,0.652495]);

    %% Robot Declaration
    schunk_dh = [0,     0,   0,     0,   0,      0,  0;  % Theta
                 0.3,   0,   0.328, 0,   0.2765, 0,  0.40049; % d
                 0,     0,   0,     0,   0,      0,  0;       % a
                -pi2,   pi2,-pi2,   pi2,-pi2,    pi2,0;       % Alpha
                 repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,7);  % Type of joint                                      
                ];
    schunk = DQ_SerialManipulatorDH(schunk_dh);
    dofs = schunk.get_dim_configuration_space();

    % Auxiliar variables
    identity = eye(8,8);
    
    
    %% start visualization
    plot(schunk,q);
    hold on;
    plot(xd);

    x_error = vec8(DQ(inf));
    %% Loops for a given amount of steps
    while (norm(x_error) > convergence_tolerance)

        x = schunk.fkm(q);
        J = schunk.pose_jacobian(q);

        %% Damping Calculation

        [U,S,V] = svd(J);
        sigma_min = S(dofs,dofs);
        u_m = U(:,dofs);

        lambda = lambda_max;
        if sigma_min < epsilon
            lambda = (1-(sigma_min/epsilon)^2)*(lambda_max^2);
        end

        %% Jacobian Inversion
        J_inv = (J')/(J*(J') + (beta*beta)*identity + (lambda*lambda)*u_m*(u_m'));

        %% Error Calculation
        % Base displacement invariant error
        % x_error = vec8(1 - x'*xd);
        % Euclidean Distance
        x_error = vec8(xd - x);    

        delta_thetas = J_inv*kp*x_error;
        q = q + delta_thetas;
        % visualization
        plot(schunk,q);
        drawnow;        
    end
end

