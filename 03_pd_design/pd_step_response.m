%% Clear the workspace
clc; clear;

%% System transfer function
s = tf('s');
G = 0.11 / (s * (s + 0.556));  % transfer function

%% PD controller gains
Kp = 400;     % proportional gain
Kd = 300;     % derivative gain

%% PD controller Gc(s)
Gc = Kp + Kd * s;  % Gc(s) = Kp + Kd*s

%% Closed loop with unity feedback
T = feedback(Gc * G, 1);  % closed loop: Gc*G / (1 + Gc*G)

%% Step response analysis
[y, t] = step(T);         % unit step response
S = stepinfo(T);          % response characteristics

OS = S.Overshoot;         % overshoot (%)
Ts = S.SettlingTime;      % settling time
ess = abs(1 - y(end));    % steady-state error (ess)

%% Plot the system response
figure('Color','w');
plot(t, y, 'b', 'LineWidth', 1.5); grid on;

% Add settling band and settling point
hold on;
plot([0 Ts], [1.02 1.02], 'r--');  % upper 2% limit
plot([0 Ts], [0.98 0.98], 'r--');  % lower 2% limit
plot(Ts, 1, 'ko', 'MarkerFaceColor','r');  % settling point

xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
title('Step response with PD controller', 'FontSize', 14);
xlim([0 max(t)]);
ylim([0 1.2]);

%% Print results to the command window
fprintf('PD controller with Kp = %.2f, Kd = %.2f\n', Kp, Kd);
fprintf('   Overshoot: %.2f%%\n', OS);
fprintf('   Settling time (2%%): %.3f sec\n', Ts);
fprintf('   Steady-state error (ess): %.4f\n', ess);
