function [xL, yL] = leader_chaotic(t)
% Scenario 5: chaotic motion
    xL = 9 * (sin(10*t) .* cos(6*t)); % chaotic x component
    yL = 9 * (cos(12*t) .* sin(8*t));% chaotic y component
end
