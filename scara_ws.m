function scara_ws(th1_lim, th2_lim, L1, L2, x0, y0, r0)
    % Function to compute and plot the joint space and workspace of a SCARA robot.
    % Inputs:
    %   th1_lim - limits of joint 1 in degrees [min, max]
    %   th2_lim - limits of joint 2 in degrees [min, max]
    %   L1 - length of the first link
    %   L2 - length of the second link
    %   x0 - x-coordinate of the obstacle center
    %   y0 - y-coordinate of the obstacle center
    %   r0 - radius of the obstacle

    % Convert the joint limits from degrees to radians
    th1_lim_rad = deg2rad(th1_lim);
    th2_lim_rad = deg2rad(th2_lim);

    %resolution of joint span
    num_points=500;
    
    % Generate the span of joint angles
    theta2 = linspace(th2_lim_rad(1), th2_lim_rad(2), num_points);
    theta1 = linspace(th1_lim_rad(1), th1_lim_rad(2), num_points);


    % No obstacle case
    if nargin == 4
        % Compute the edges of the joint space without considering obstacles
        edges=edges_computation(theta1,theta2);
    else
        % Initialize edges array for 10 edges when considering obstacles
        %edges = zeros(num_points, 2, 10);

        % Evaluate collision between the obstacle and link 2
        q2max = max(abs(th2_lim_rad(1)), abs(th2_lim_rad(2)));
        xee_min = L1 + L2 * cos(q2max);
        yee_min = L2 * sin(q2max);
        % Minimum distance of the end-effector from the center
        radiusPlcm = sqrt(xee_min^2 + yee_min^2);
        % Distance of the obstacle's furthest point from the center
        obsDist = sqrt(x0^2 + y0^2) + r0;
        if (obsDist > radiusPlcm && obsDist < (L1 + L2))
            error("Obstacle may collide with link 2")
        end
        %compute radius of allowed area
        

        % Compute the angle of the obstacle with respect to base frame
        theta_obs=atan2(y0,x0);
        % Distance between the origin and the center of the obstacle
        d=sqrt(y0^2+x0^2);
        alfa=asin(r0/d);
        thobs_lim=[theta_obs-alfa;theta_obs+alfa];
        rad2deg(thobs_lim)
    
        % Consider the new boundary of the joint space as the obstacle angle threshold
        if (obsDist>(L1+L2))
            edges= edges_computation(theta1,theta2);

        elseif thobs_lim(1)<th1_lim_rad(1)
            if thobs_lim(2)<th1_lim_rad(1)
                edges= edges_computation(theta1,theta2);
            else
                % The excluded joint space region overflows to the left
                theta1 = linspace(thobs_lim(2), th1_lim_rad(2), num_points);
                edges = edges_computation (theta1,theta2);
            end

        elseif thobs_lim(2)>th2_lim_rad(2)
            if thobs_lim(1)>th1_lim_rad(2)
                edges= edges_computation(theta1,theta2);
            else
                % The excluded joint space region overflows to the right
                theta1 = linspace(th1_lim_rad(1), thobs_lim(1), num_points);
                edges = edges_computation (theta1,theta2);
            end

        else
            % The excluded joint space region remains inside the span of the first joint
            edges = edges_computation (theta1,theta2,thobs_lim);
        end

    end

    % Plot the joint space in degrees
    figure;
    hold on;
    for i = 1:size(edges,3)
        plot(edges(:, 1, i) * (180/pi), edges(:, 2, i) * (180/pi), 'k', 'LineWidth', 1.5); % Contours in black
    end
    xlabel('Theta 1 (degrees)');
    ylabel('Theta 2 (degrees)');
    title('SCARA Joint Space');
    grid on;
    hold off;

    % Initialize matrices for Cartesian coordinates
    X = [];
    Y = [];

    % Calculate the Cartesian coordinates for each combination of theta1 and theta2
    for i = 1:size(edges, 3)
        for j = 1:num_points
            % Forward kinematics equations
            X(j,i) = L1 * cos(edges(j, 1, i)) + L2 * cos(edges(j, 1, i) + edges(j, 2, i));
            Y(j,i) = L1 * sin(edges(j, 1, i)) + L2 * sin(edges(j, 1, i) + edges(j, 2, i));
        end
    end

    % Plot the Cartesian workspace in radians
    figure;
    hold on;
    plot(X, Y, 'k', 'LineWidth', 1.5); % Contours in black
    
    % Add circular obstacle
    theta_obs = linspace(0, 2*pi, 100); % Angles to describe the circle
    x_circle = x0 + r0 * cos(theta_obs);
    y_circle = y0 + r0 * sin(theta_obs);
    plot(x_circle, y_circle, 'r--', 'LineWidth', 1.5); % Obstacle in red
    
    % Labels and title
    xlabel('X (m)');
    ylabel('Y (m)');
    title('SCARA Robot Workspace with Circular Obstacle');
    grid on;
    axis equal; % Equal scale for X and Y axes
    hold off;

end