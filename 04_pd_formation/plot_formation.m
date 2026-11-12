function plot_formation(xL, yL, xA1, yA1, xA2, yA2, R)
    k = length(xL);
    clf; hold on; grid on;

    % circle around the leader
    theta = linspace(0, 2*pi, 200);
    cx = R*cos(theta) + xL(k);
    cy = R*sin(theta) + yL(k);
    plot(cx, cy, 'k:');

    % dashed trajectories
    plot(xL, yL, 'r--');
    if ~isempty(xA1), plot(xA1, yA1, 'b--'); end
    if ~isempty(xA2), plot(xA2, yA2, 'k--'); end  % linearized model in black

    % positions
    plot(xL(k), yL(k), 'ro', 'MarkerFaceColor', 'r');
    text(xL(k), yL(k)+0.015, 'L', 'Color', 'r');

    if ~isempty(xA1)
        plot(xA1(k), yA1(k), 'bo', 'MarkerFaceColor', 'b');
        text(xA1(k), yA1(k)+0.015, 'A1', 'Color', 'b');
    end

    if ~isempty(xA2)
        plot(xA2(k), yA2(k), 'ko', 'MarkerFaceColor', 'k');  % linearized model in black
        text(xA2(k), yA2(k)+0.015, 'A2', 'Color', 'k');
    end

    axis equal;
    axis([-30 30 -30 30] + [xL(k), xL(k), yL(k), yL(k)]);

    % title depends on what is drawn
    if ~isempty(xA1) && isempty(xA2)
        title('PD nonlinear', 'FontSize', 12, 'FontWeight', 'bold');
    elseif isempty(xA1) && ~isempty(xA2)
        title('PD linearized', 'FontSize', 12, 'FontWeight', 'bold');
    else
        title('PD control - A1 & A2', 'FontSize', 12, 'FontWeight', 'bold');
    end
end
