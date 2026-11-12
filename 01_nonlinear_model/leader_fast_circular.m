function [xL, yL] = leader_fast_circular(t)
% Scenario 4: circular motion with high angular velocity
    R = 9; % radius of the circular path
    omega = 3*pi;% high angular velocity
    xL = R * cos(omega * t);% x position
    yL = R * sin(omega * t);% y position
end
