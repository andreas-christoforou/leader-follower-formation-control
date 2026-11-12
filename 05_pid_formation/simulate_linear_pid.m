function [xL, yL, xA, yA] = simulate_linear_pid(L_fun, R, theta0)
    T = 10; dt = 0.01;
    t = 0:dt:T; N = length(t);

    % PID gains
    Kp = 500; Ki = 9.09; Kd = 340;

    % Linearized A, B
    A = [0 1 0 0;
         0 -0.556 0 0;
         0 0 0 1;
         0 0 0 -0.556];

    B = [0 0;
         0.11 0;
         0 0;
         0 0.11];

    % States [r, r_dot, theta, theta_dot]
    x = zeros(4, N);
    ei_r = 0; ei_theta = 0;

    % Leader and A2 positions
    xL = zeros(1,N); yL = zeros(1,N);
    xA = zeros(1,N); yA = zeros(1,N);

    % Initial leader position (center of the moving circle)
    posL = L_fun(0);
    xL(1) = posL(1); yL(1) = posL(2);

    % Initial A2 position: on the circle, at angle θ = 360°
    r0 = R;
    theta_init = theta0;
    x(1,1) = r0;
    x(3,1) = theta_init;

    xA(1) = xL(1) + r0 * cos(theta_init);
    yA(1) = yL(1) + r0 * sin(theta_init);

    for k = 1:N
        posL = L_fun(t(k));
        xL(k) = posL(1); yL(k) = posL(2);

        % PID errors
        e_r = x(1,k) - R;
        ei_r = ei_r + e_r * dt;

        e_theta = wrapToPi(x(3,k) - theta0);
        ei_theta = ei_theta + e_theta * dt;

        v_r = x(2,k);
        v_theta = x(4,k);

        % PID control
        Fr = -Kp * e_r - Kd * v_r - Ki * ei_r;
        Fn = -Kp * e_theta - Kd * v_theta - Ki * ei_theta;

        theta = x(3,k);
        fx = Fr * cos(theta) - Fn * sin(theta);
        fy = Fr * sin(theta) + Fn * cos(theta);
        u = [fx; fy];

        % Euler integration
        if k < N
            x(:,k+1) = x(:,k) + dt * (A*x(:,k) + B*u);
            r_next = x(1,k+1);
            theta_next = x(3,k+1);
            xA(k+1) = xL(k) + r_next * cos(theta_next);
            yA(k+1) = yL(k) + r_next * sin(theta_next);
        end
    end
end
