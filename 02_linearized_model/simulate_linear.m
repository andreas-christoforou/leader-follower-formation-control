% Linearized system without a controller.
% Vehicle A receives forces Fr and Fn and follows the leader.

clc; clear; close all;

%% Physical parameters of the vehicle
m = 9;              % mass (kg)
c = 5;              % damping coefficient
R = 9;              % desired radius (distance from the leader)

%% Simulation time parameters
dt = 0.01;         % time step (s)
tf = 10;            % total simulation time (s)
t = 0:dt:tf;        % time vector
N = length(t);      % number of steps
th = linspace(0, 2*pi, 100); % for drawing the circle
%% Leader trajectories (scenarios)
% Each function returns [xL; yL] for every t
raw_trajectories = {
    @(t) [t;0],                                                 % Scenario 1: straight line
    @(t) [0.5*t.^2; 0],                                         % Scenario 2: accelerating
    @(t) [20*cos(0.2*t); 20*sin(0.2*t)],                        % Scenario 3: slow circle
    @(t) [8*cos(0.6*t); 8*sin(0.6*t)],                          % Scenario 4: fast circle
    @(t) [10*sin(1.5*t).*cos(0.5*t); 10*cos(2.2*t).*sin(0.7*t)] % Scenario 5: chaotic
};

%% Force inputs (Fr, Fn) per scenario
% open-loop force profile for each scenario
forces = {
    @(t) [5 + 2*sin(0.2*t); 5 + 2*cos(0.2*t)],           % Scenario 1
    @(t) [5 + t; 15 - t],                                % Scenario 2
    @(t) [5 + 2*sin(t); 5 + 2*cos(t)],                   % Scenario 3
    @(t) [2 + 2*sin(0.3*t); 2 + 2*cos(0.4*t)],           % Scenario 4
    @(t) [3*sin(0.1*t.^2); 33*cos(0.2*t.^1.5)]           % Scenario 5
};

%% Run the 5 scenarios
for scenario = 1:5
    % Shift the leader trajectory so it starts at (0,0)
    Lraw = raw_trajectories{scenario};
    L0 = Lraw(0);
    Lfun = @(t) Lraw(t) - L0;

    %  Pick the force profile for this scenario
    Ffun = forces{scenario};

    % Initialize position/velocity variables
    xL = zeros(1, N); yL = zeros(1, N);          % leader position
    xA = zeros(1, N); yA = zeros(1, N);         % position of A
    vxA = zeros(1, N); vyA = zeros(1, N);       % velocity of A

    % Initial position of A on the circle at 0/360 degrees
    theta0 = deg2rad(0);
    xA(1) = R * cos(theta0);
    yA(1) = R * sin(theta0);

    %% Dynamics simulation (Euler integration)
    for k = 1:N-1
        tk = t(k);                     % current time
        Lk = Lfun(tk);                  % leader position
        xL(k) = Lk(1); yL(k) = Lk(2);

        % Position relative to the leader
        dx = xA(k) - xL(k);
        dy = yA(k) - yL(k);
        r_now = sqrt(dx^2 + dy^2);     % distance (optional, for metrics)


        % Forces Fr and Fn
        F = Ffun(tk);
        Fr = F(1); Fn = F(2);

        % Acceleration from the linear model
        ax = (Fr - c * vxA(k)) / m;
        ay = (Fn - c * vyA(k)) / m;

        % Euler: update velocity and position
        vxA(k+1) = vxA(k) + ax * dt;
        vyA(k+1) = vyA(k) + ay * dt;
        xA(k+1) = xA(k) + vxA(k+1) * dt;
        yA(k+1) = yA(k) + vyA(k+1) * dt;
    end

    % Last leader position
    Lk = Lfun(t(end));
    xL(N) = Lk(1); yL(N) = Lk(2);

    %% Animation
    figure('Color','w','Name',sprintf('Linearized model, no controller - Scenario %d',scenario));
    for k = 1:10:N
        clf; hold on;

        % target circle
        plot(xL(k) + R*cos(th), yL(k) + R*sin(th), 'k:');

        % draw the leader
        plot(xL(k), yL(k), 'ro', 'MarkerFaceColor','r');
        text(xL(k), yL(k)-R*0.05, 'L', 'Color','r','FontWeight','bold');

        % draw vehicle A
        plot(xA(k), yA(k), 'bo', 'MarkerSize', 6, 'MarkerFaceColor','b');
        text(xA(k), yA(k)+R*0.05, 'A', 'Color','b','FontWeight','bold');

        % trajectories
        plot(xL(1:k), yL(1:k), 'r--'); % leader trajectory
        plot(xA(1:k), yA(1:k), 'b--'); % vehicle A trajectory

        % Plot settings
        axis equal;
        axis([-20 20 -20 20]);
        title(sprintf('No controller | Scenario %d | t=%.2fs', scenario, t(k)));
        xlabel('x'); ylabel('y');
        grid on; drawnow;
    end
end
