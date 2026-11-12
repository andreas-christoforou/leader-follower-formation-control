%% Clear the workspace
clc; clear; close all;

%% Leader trajectory scenarios
% Each scenario has a different leader trajectory
titles = {
    '1: Fast straight line',
    '2: Very fast straight line',
    '3: Circle',
    '4: Faster circle',
    '5: Spiral oscillating motion'
};

% Functions that return the leader position [xL; yL] for each t
trajectories = {
   @(t) [2*t; 0],
   @(t) [5*t; 0],
   @(t) [10*cos(0.6*t); 10*sin(0.6*t)],
   @(t) [15*cos(0.9*t); 15*sin(0.9*t)],
   @(t) [10*t + 5*cos(5*t); 10*sin(5*t)]
};

%% Circular formation parameters
R = 9;           % circle radius
theta0 = 0;      % initial angle in rad

%% Run all 5 scenarios
for scenario = 1:5
    fprintf('\n--- PID nonlinear model - Scenario %d ---\n', scenario);
    L_fun = trajectories{scenario};  % pick the trajectory

    % Simulate with the PID controller
    [xL, yL, xA, yA, ess, e_r, e_theta, t] = simulate_nonlinear_pid(L_fun, R, theta0);

    %% Show the animation (trajectory)
    dt = 0.01;
    for k = 1:10:length(xL)
        tic;
        plot_pid_trajectory(xL(1:k), yL(1:k), xA(1:k), yA(1:k), R, scenario);
        title(sprintf('PID nonlinear - Scenario %d', scenario), 'FontWeight', 'bold');
        drawnow;
        pause(max(0, dt*10 - toc));  % constant animation speed
    end

    % Print the final radius error
    fprintf('ess : %.4f\n', ess);

    %% Plot radius and angle errors
    figure('Name', sprintf('Errors - Scenario %d', scenario));
    subplot(2,1,1);
    plot(t, e_r, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('e_r (m)');
    title(sprintf('PID radius error - Scenario %d', scenario));
    grid on;

    subplot(2,1,2);
    plot(t, e_theta, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('e_θ (rad)');
    title(sprintf('PID angle error - Scenario %d', scenario));
    grid on;

    pause(5);  % short pause between scenarios
end

%% === Nonlinear PID simulation ===
function [xL, yL, xA, yA, ess, e_r_vals, e_theta_vals, t] = simulate_nonlinear_pid(L_fun, R, theta0)
    % Nonlinear motion of A1 with a PID controller

    T = 10; dt = 0.01;
    t = 0:dt:T; N = length(t);

    % System parameters
    m = 9; c = 5;

    % PID gains (same for Fr and Fn)
    Kp = 500;
    Ki = 9.09;
    Kd = 340;

    % Initialize variables
    xL = zeros(1,N); yL = zeros(1,N);
    xA = zeros(1,N); yA = zeros(1,N);
    vx = zeros(1,N); vy = zeros(1,N);

    r_vals = zeros(1,N);
    e_r_vals = zeros(1,N);
    e_theta_vals = zeros(1,N);

    % Initial position: A on the circle centered at the leader
    posL = L_fun(0);
    xL(1) = posL(1); yL(1) = posL(2);
    xA(1) = xL(1) + R * cos(theta0);
    yA(1) = yL(1) + R * sin(theta0);

    % Initial values of the integral terms
    ei_r = 0; ei_theta = 0;

    for k = 1:N
        % Leader position at time t(k)
        posL = L_fun(t(k));
        xL(k) = posL(1); yL(k) = posL(2);

        % Relative position
        dx = xA(k) - xL(k);
        dy = yA(k) - yL(k);
        r = hypot(dx, dy);
        theta = atan2(dy, dx);

        % Radius error and radial velocity
        e_r = r - R;
        v_r = (dx * vx(k) + dy * vy(k)) / r;
        ei_r = ei_r + e_r * dt;  % integral of the error

        % Angle error and angular velocity
        e_theta = wrapToPi(theta - theta0);
        v_theta = (-dy * vx(k) + dx * vy(k)) / r;
        ei_theta = ei_theta + e_theta * dt;

        % Store the errors
        r_vals(k) = r;
        e_r_vals(k) = e_r;
        e_theta_vals(k) = e_theta;

        % PID controller for Fr and Fn
        Fr = -Kp * e_r - Kd * v_r - Ki * ei_r;
        Fn = -Kp * e_theta * R - Kd * v_theta - Ki * R * ei_theta;

        % convert to fx/fy in the Cartesian frame
        fx = Fr * cos(theta) - Fn * sin(theta);
        fy = Fr * sin(theta) + Fn * cos(theta);

        % accelerations and Euler step
        ax = (fx - c * vx(k)) / m;
        ay = (fy - c * vy(k)) / m;

        if k < N
            vx(k+1) = vx(k) + dt * ax;
            vy(k+1) = vy(k) + dt * ay;
            xA(k+1) = xA(k) + dt * vx(k+1);
            yA(k+1) = yA(k) + dt * vy(k+1);
        end
    end

    % Final radius error (ess)
    ess = abs(r_vals(end) - R);
end

%% === Plotting function ===
function plot_pid_trajectory(xL, yL, xA, yA, R, scenario)
    % Draw the A1 and leader trajectories for each scenario
    clf; hold on; grid on;
    k = length(xL);

    % Reference circle
    theta = linspace(0, 2*pi, 200);
    cx = R * cos(theta) + xL(k);
    cy = R * sin(theta) + yL(k);
    plot(cx, cy, 'k:');                     % reference circle

    plot(xL, yL, 'r--');                    % leader trajectory
    plot(xA, yA, 'b--');                    % A1 trajectory

    plot(xL(k), yL(k), 'ro', 'MarkerFaceColor', 'r');
    text(xL(k), yL(k)+0.3, 'L', 'Color', 'r');

    plot(xA(k), yA(k), 'bo', 'MarkerFaceColor', 'b');
    text(xA(k), yA(k)+0.3, 'A1', 'Color', 'b');

    % Zoom out so the view follows the leader
    axis equal;
    axis([-30 30 -30 30] + [xL(k), xL(k), yL(k), yL(k)]);
end
