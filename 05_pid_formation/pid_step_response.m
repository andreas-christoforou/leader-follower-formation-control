clc; clear;

% System transfer function
s = tf('s');
G = 0.11 / (s * (s + 0.556));  % plant to control

% PID gains
Kp = 500; Ki = 9.09; Kd = 340;

% PID controller: Gc(s) = Kp + Kd*s + Ki/s
Gc = Kp + Kd*s + Ki/s;

% Closed loop
T = feedback(Gc * G, 1);

% Response
[y, t] = step(T);
e = abs(1 - y);           % error over time
ess = abs(1 - y(end));    % steady-state error

% Response characteristics
S = stepinfo(T);
OS = S.Overshoot;         % overshoot (%)
Ts = S.SettlingTime;      % settling time (2%)

% Print results
fprintf('PID with Kp = %.2f, Kd = %.2f, Ki = %.2f\n', Kp, Kd, Ki);
fprintf('Overshoot          = %.2f %%\n', OS);
fprintf('Settling time      = %.3f sec\n', Ts);
fprintf('ess (final error)  = %.5f\n', ess);

% Response plot
figure('Color','w');
plot(t, y, 'b', 'LineWidth', 1.5); grid on;
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
title('Step response with tuned PID', 'FontSize', 14);
xlim([0 max(t)]);
ylim([0 1.2]);
