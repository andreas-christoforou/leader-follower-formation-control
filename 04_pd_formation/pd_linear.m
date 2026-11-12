%% Clear the workspace
clc; clear; close all;

%% Leader trajectories (5 scenarios)
% Each function returns [xL; yL] for one scenario
trajectories = {
    @(t) [2*t; 0],                                       % Scenario 1: fast straight line
    @(t) [5*t; 0],                                       % Scenario 2: very fast straight line
    @(t) [10*cos(0.6*t); 10*sin(0.6*t)],                 % Scenario 3: circle
    @(t) [15*cos(0.9*t); 15*sin(0.9*t)],                 % Scenario 4: fast circle
    @(t) [10*t + 5*cos(5*t); 10*sin(5*t)]                % Scenario 5: spiral oscillating motion
};

R = 9;               % circle radius
theta0 = 0;          % initial angle (rad)

%% Run the simulation for each scenario
for scenario = 1:5
    fprintf('\n--- PD linearized model - Scenario %d ---\n', scenario);
    L_fun = trajectories{scenario};

    % Simulate the linearized model
    [xL, yL, xA, yA] = simulate_linear_pd(L_fun, R, theta0);

    % Show the animation
    dt = 0.01;
    for k = 1:10:length(xL)
        tic;
        plot_formation(xL(1:k), yL(1:k), [], [], xA(1:k), yA(1:k), R);
        title(sprintf('PD linearized - Scenario %d', scenario), 'FontWeight', 'bold');
        drawnow;
        pause(max(0, dt*10 - toc));  % animation timing
    end

    pause(3);  % short pause between scenarios
end

%% Linearized system simulation
function [xL, yL, xA, yA] = simulate_linear_pd(L_fun, R, theta0)
    T = 10; dt = 0.01;
    t = 0:dt:T; N = length(t);

    % Linearized model: x_dot = A*x + B*u
    A = [0 1 0 0;
         0 -0.556 0 0;
         0 0 0 1;
         0 0 0 -0.556];

    B = [0 0;
         0.11 0;
         0 0;
         0 0.11];

    % PD gains
    Kp_r = 400;   Kd_r = 300;   % radius
    Kp_t = 400;   Kd_t = 300;   % angle

    % State x = [r; r_dot; theta; theta_dot]
    x = zeros(4, N);

    % Leader and A positions
    xL = zeros(1,N); yL = zeros(1,N);
    xA = zeros(1,N); yA = zeros(1,N);

    % Initial leader position and A on the circle
    posL = L_fun(0);
    xL(1) = posL(1); yL(1) = posL(2);
    xA(1) = xL(1) + R * cos(theta0);
    yA(1) = yL(1) + R * sin(theta0);

    % initial radius and angle
    dx0 = xA(1) - xL(1);
    dy0 = yA(1) - yL(1);
    r0 = sqrt(dx0^2 + dy0^2);
    theta0_actual = atan2(dy0, dx0);

    % Initial state x = [r; r_dot; theta; theta_dot]
    x(:,1) = [r0; 0; theta0_actual; 0];

    %% Euler simulation x_dot = A*x + B*u
    for k = 1:N
        % Leader position
        posL = L_fun(t(k));
        xL(k) = posL(1); yL(k) = posL(2);

        % Radius and angle errors + velocities
        e_r = x(1,k) - R;
        v_r = x(2,k);
        e_theta = wrapToPi(x(3,k) - theta0);
        v_theta = x(4,k);

        % PD control
        Fr = -Kp_r * e_r - Kd_r * v_r;
        Fn = -Kp_t * e_theta - Kd_t * v_theta;

        % convert to fx, fy
        theta_k = x(3,k);
        fx = Fr * cos(theta_k) - Fn * sin(theta_k);
        fy = Fr * sin(theta_k) + Fn * cos(theta_k);

        u = [fx; fy];  % system input u

        % Euler integration
        if k < N
            x(:,k+1) = x(:,k) + dt * (A * x(:,k) + B * u);

            % position of A in world coordinates
            r = x(1,k+1); theta = x(3,k+1);
            xA(k+1) = xL(k) + r * cos(theta);
            yA(k+1) = yL(k) + r * sin(theta);
        end
    end
end
