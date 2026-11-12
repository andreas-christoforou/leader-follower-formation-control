%% Draws the leader and vehicles A1/A2
function plot_formation_pid(xL, yL, xA1, yA1, xA2, yA2, R)
    k = length(xL);          % current step index
    clf; hold on; grid on;   % clear the figure and turn on the grid

    %% Reference circle around the leader
    theta = linspace(0, 2*pi, 200);
    cx = R * cos(theta) + xL(k);  % circle x coordinates
    cy = R * sin(theta) + yL(k);  % circle y coordinates
    plot(cx, cy, 'k:');           % dotted circle centered at the leader

    %% Leader trajectory and position
    plot(xL, yL, 'r--');                                   % red dashed trajectory
    plot(xL(k), yL(k), 'ro', 'MarkerFaceColor', 'r');      % red marker (current)
    text(xL(k), yL(k)+0.4, 'L', 'Color', 'r');             % "L" label above the leader

    %% A1 trajectory and position (blue)
    if ~isempty(xA1)
        plot(xA1, yA1, 'b--');                              % blue dashed trajectory
        plot(xA1(k), yA1(k), 'bo', 'MarkerFaceColor', 'b');% blue marker
        text(xA1(k), yA1(k)+0.4, 'A1', 'Color', 'b');       % "A1" label
    end

    %% A2 trajectory and position (black)
    if ~isempty(xA2)
        plot(xA2, yA2, 'k--');                              % black dashed trajectory
        plot(xA2(k), yA2(k), 'ko', 'MarkerFaceColor', 'k');% black marker
        text(xA2(k), yA2(k)+0.4, 'A2', 'Color', 'k');       % "A2" label
    end

    %% Axis limits around the leader
    axis equal;
    axis([-15 15 -15 15] + [xL(k), xL(k), yL(k), yL(k)]);  % moving view
end
