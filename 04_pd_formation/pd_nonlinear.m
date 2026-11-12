%% Clear the workspace
clc; clear; close all;

%% Leader trajectory scenarios
titles = {
    '1: Fast straight line',
    '2: Very fast straight line',
    '3: Circle',
    '4: Faster circle',
    '5: Spiral oscillating motion'
};

trajectories = {
   @(t) [2*t; 0],                                       % Scenario 1: fast straight line
    @(t) [5*t; 0],                                       % Scenario 2: very fast straight line
    @(t) [10*cos(0.6*t); 10*sin(0.6*t)],                 % Scenario 3: circle
    @(t) [15*cos(0.9*t); 15*sin(0.9*t)],                 % Scenario 4: fast circle
    @(t) [10*t + 5*cos(5*t); 10*sin(5*t)]                % Scenario 5: spiral oscillating motion
};

%% System parameters
R = 9;            % radius of the circle to keep
theta0 = 0;       % initial angle

%% Run the 5 scenarios
for scenario = 1:5
    fprintf('\n--- PD nonlinear model - Scenario %d ---\n', scenario);
    L_fun = trajectories{scenario};

    [xL, yL, xA, yA, ess, e_r, e_theta, t] = simulate_nonlinear_pd(L_fun, R, theta0);

    %% Show the animation
    dt = 0.01;
    for k = 1:10:length(xL)
        tic;
        plot_formation(xL(1:k), yL(1:k), xA(1:k), yA(1:k), [], [], R);
        title(sprintf('PD nonlinear - Scenario %d', scenario), 'FontWeight', 'bold');
        drawnow;
        pause(max(0, dt*10 - toc));  % constant animation speed
    end

    fprintf('ess : %.4f\n', ess);  % final radius error

    %% Plot radius and angle errors
    figure('Name', sprintf('Errors - Scenario %d', scenario));
    subplot(2,1,1);
    plot(t, e_r, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('e_r (m)');
    title(sprintf('PD radius error - Scenario %d', scenario));
    grid on;

    subplot(2,1,2);
    plot(t, e_theta, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('e_θ (rad)');
    title(sprintf('PD angle error - Scenario %d', scenario));
    grid on;

    pause(5);  % 5 second pause after each scenario
end

%% Nonlinear system simulation with PD control
function [xL, yL, xA, yA, ess, e_r_vals, e_theta_vals, t] = simulate_nonlinear_pd(L_fun, R, theta0)
    % basic dynamics with Euler integration and a PD controller

    T = 10; dt = 0.01;
    t = 0:dt:T; N = length(t);

    % system parameters
    m = 9; c = 5;

    % PD gains
    Kp_r = 400; Kd_r = 300; % radius
    Kp_t = 400; Kd_t = 300; % angle

    % initialize variables
    xL = zeros(1,N); yL = zeros(1,N);
    xA = zeros(1,N); yA = zeros(1,N);
    vx = zeros(1,N); vy = zeros(1,N);
    r_vals = zeros(1,N);
    e_r_vals = zeros(1,N);
    e_theta_vals = zeros(1,N);

    % initial position of A on the circle centered at the leader
    posL = L_fun(0);
    xL(1) = posL(1); yL(1) = posL(2);
    xA(1) = xL(1) + R * cos(theta0);
    yA(1) = yL(1) + R * sin(theta0);

    % Euler integration
    for k = 1:N
        posL = L_fun(t(k));
        xL(k) = posL(1); yL(k) = posL(2);

        dx = xA(k) - xL(k);
        dy = yA(k) - yL(k);
        r = hypot(dx, dy);  % distance of A from the leader
        theta = atan2(dy, dx);  % angle relative to the leader

        % radius error and radial velocity
        e_r = r - R;
        v_r = (dx * vx(k) + dy * vy(k)) / r;

        % angle error and angular velocity
        e_theta = wrapToPi(theta - theta0);
        v_theta = (-dy * vx(k) + dx * vy(k)) / r;

        r_vals(k) = r;
        e_r_vals(k) = e_r;
        e_theta_vals(k) = e_theta;

        % PD forces
        Fr = -Kp_r * e_r - Kd_r * v_r;
        Fn = -Kp_t * e_theta * R - Kd_t * v_theta;

        % convert to Cartesian frame
        fx = Fr * cos(theta) - Fn * sin(theta);
        fy = Fr * sin(theta) + Fn * cos(theta);

        % accelerations and Euler integration
        ax = (fx - c * vx(k)) / m;
        ay = (fy - c * vy(k)) / m;

        if k < N
            vx(k+1) = vx(k) + dt * ax;
            vy(k+1) = vy(k) + dt * ay;
            xA(k+1) = xA(k) + dt * vx(k+1);
            yA(k+1) = yA(k) + dt * vy(k+1);
        end
    end

    ess = abs(r_vals(end) - R);  % final radius error
end
