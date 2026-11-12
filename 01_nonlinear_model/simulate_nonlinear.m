% Nonlinear simulation of a vehicle A that follows a moving leader
function simulate_nonlinear()
    % Physical parameters of the system
    m = 9;          % vehicle mass (kg)
    c = 5;          % damping coefficient
    dt = 0.01;      % integration time step (s)
    steps = 10000;  % maximum number of simulation steps

    % Initial position of the vehicle on the leader's circle,
    % at 0/360 degrees
    r_ref = 9;
    theta0 = deg2rad(360);
    x0 = r_ref * cos(theta0);
    y0 = r_ref * sin(theta0);

    % 5 different scenarios for the leader's motion
    run_scenario(@leader_slow_linear,     'Scenario 1 - Slow straight line',  x0, y0, r_ref, m, c, dt, steps);
    run_scenario(@leader_fast_linear,     'Scenario 2 - Fast straight line',  x0, y0, r_ref, m, c, dt, steps);
    run_scenario(@leader_slow_circular,   'Scenario 3 - Slow circle',         x0, y0, r_ref, m, c, dt, steps);
    run_scenario(@leader_fast_circular,   'Scenario 4 - Fast circle',         x0, y0, r_ref, m, c, dt, steps);
    run_scenario(@leader_chaotic,         'Scenario 5 - Chaotic motion',      x0, y0, r_ref, m, c, dt, steps);
end

function run_scenario(leader_fun, title_str, x0, y0, r_ref, m, c, dt, steps)
% Runs one simulation scenario of vehicle A around the leader


% State matrix initialization: [x; xdot; y; ydot]
    X = zeros(4, steps);
    X(:,1) = [x0; 0; y0; 0]; % initial position/velocity

    % Store the leader's path at every step
    xL_vec = zeros(1, steps);
    yL_vec = zeros(1, steps);

    % Open the plot window
    figure('Name', title_str, 'NumberTitle', 'off');
    k = 1;
    tic; % start the timer (run for 10 seconds)

    while toc < 10 && k < steps
        t = (k-1)*dt;             % simulation time
        [xL, yL] = leader_fun(t); % current leader position

        % current position and velocity of A
        x = X(1,k); xdot = X(2,k);
        y = X(3,k); ydot = X(4,k);

        % relative position and radius
        dx = x - xL;
        dy = y - yL;
        r = sqrt(dx^2 + dy^2 + 1e-12); % small offset to avoid division by zero

        % direction from the leader to A
        cos_theta = dx / r;
        sin_theta = dy / r;

        % rotation matrix RL (converts forces from polar to Cartesian)
        RL = [cos_theta -sin_theta;
              sin_theta  cos_theta];

        % constant forces applied to vehicle A (polar frame)
        F = [110; 160];% Fr=110 (radial), Fn=160 (tangential)

        % acceleration from F/m plus damping
        acc = (1/m) * RL * F - (c/m) * [xdot; ydot];

        % update velocity and position with Euler's method
        X(2,k+1) = xdot + dt * acc(1);
        X(1,k+1) = x + dt * X(2,k+1);
        X(4,k+1) = ydot + dt * acc(2);
        X(3,k+1) = y + dt * X(4,k+1);

        % store the leader's position
        xL_vec(k) = xL;
        yL_vec(k) = yL;

        % draw the animation frame
        cla; hold on;
        theta_circ = linspace(0, 2*pi, 100);
        plot(xL + r_ref*cos(theta_circ), yL + r_ref*sin(theta_circ), '--', 'Color', [0.7 0.7 0.7]);
        plot(xL, yL, 'ro', 'MarkerFaceColor','r');
        text(xL + 0.4, yL + 0.4, 'L', 'FontSize', 10, 'Color', 'r');
        plot(x, y, 'bo', 'MarkerFaceColor','b');
        text(x + 0.4, y + 0.4, 'A', 'FontSize', 10, 'Color', 'b');
        plot(X(1,1:k), X(3,1:k), 'b:');           % path of vehicle A
        plot(xL_vec(1:k), yL_vec(1:k), 'r:');     % path of the leader
        xlim([-20, 20]); ylim([-20, 20]);
        axis equal;
        title(title_str);
        grid on;
        drawnow;
        pause(dt); % short pause for the animation

        k = k + 1;
    end

    close(gcf);
end
