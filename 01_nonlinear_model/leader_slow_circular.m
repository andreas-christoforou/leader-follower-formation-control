function [xL, yL] = leader_slow_circular(t)
% Scenario 3: circular motion with low angular velocity
    R = 9;% radius of the circular path
    omega = pi;% angular velocity
    xL = R * cos(omega * t);% x coordinate
    yL = R * sin(omega * t);% y coordinate
end
