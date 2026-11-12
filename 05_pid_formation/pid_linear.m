%% Program entry point
function pid_linear()
    clc; clear; close all;

%% Leader trajectories (5 different scenarios)
% Each function returns [xL; yL] for t, the path the leader follows
    trajectories = {
        @(t) [2*t; 0],                                   % Scenario 1: fast straight line
        @(t) [5*t; 0],                                   % Scenario 2: very fast straight line
        @(t) [6*cos(1.5*t); 6*sin(1.5*t)],               % Scenario 3: circle
        @(t) [9*cos(3*t); 9*sin(3*t)],                   % Scenario 4: fast circle
        @(t) [10*t + 5*cos(5*t); 10*sin(5*t)]            % Scenario 5: spiral oscillating motion
    };

%% Circular formation parameters
    R = 9;                        % circle radius
    theta0 = deg2rad(360);       % initial angle in rad (360/0)

%% Run the simulation for each scenario
    for scenario = 1:5
        fprintf('\n--- PID linearized model - Scenario %d ---\n', scenario);
        L_fun = trajectories{scenario};  % pick the leader trajectory

        % Call the simulation with the PID controller
        [xL, yL, xA, yA] = simulate_linear_pid(L_fun, R, theta0);

        dt = 0.01;

        %% animation
        for k = 1:10:length(xL)
            tic;

            % Plot the leader and A2
            plot_formation_pid(xL(1:k), yL(1:k), [], [], xA(1:k), yA(1:k), R);

            title(sprintf('PID linearized - Scenario %d', scenario), ...
                  'FontSize', 14, 'FontWeight', 'bold');

            % Dynamic zoom so the view follows the leader
            axis equal;
            axis([-30 30 -30 30] + [xL(k), xL(k), yL(k), yL(k)]);

            drawnow;
            pause(max(0, dt*10 - toc));  % timing control
        end

        % three second pause after each scenario
        pause(3);
    end
end
