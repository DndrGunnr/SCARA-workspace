function edges = edges_computation(theta1, theta2, thobs_lim)
    % Function to compute the edges of the joint space of the SCARA robot.
    % This function can be used for the allowable cases in the workspace computation:
    % - No obstacle
    % - Obstacle that fits inside the joint space
    % - Obstacle that partially goes out the outer joint space rectangle
    
    % Determine the number of points in the theta range
    num_points = size(theta1, 2);
    
    if nargin == 2
        % Edges layout:
        % |--------------2----------------|
        %1|--------------5----------------|3
        % |--------------4----------------|

        % This case covers both the no-obstacle case and the obstacle's
        % angles partially going beyond joint limits
        edges = zeros(num_points, 2, 5); % Initialize edges array for 5 edges

        % Define the edges based on the joint limits

        % Left vertical edge: theta1 is constant at theta1(1), theta2 varies
        edges(:, 1, 1) = theta1(1);             
        edges(:, 2, 1) = theta2;                

        % Top horizontal edge: theta2 is constant at theta2(num_points), theta1 varies
        edges(:, 1, 2) = theta1;
        edges(:, 2, 2) = theta2(num_points);
        
        % Right vertical edge: theta1 is constant at theta1(num_points), theta2 varies
        edges(:, 1, 3) = theta1(num_points);
        edges(:, 2, 3) = theta2;
        
        % Bottom horizontal edge: theta2 is constant at theta2(1), theta1 varies
        edges(:, 1, 4) = theta1;                
        edges(:, 2, 4) = theta2(1);             
        
        % Middle horizontal edge at theta2 = 0: theta1 varies, theta2 is constant at 0
        edges(:, 1, 5) = theta1;
        edges(:, 2, 5) = 0;
    else
        % Edges layout:
        % |------5-----|////|-----6-------|
        %1|------9-----|3//4|-----10------|2
        % |------7-----|////|-----8-------|

        % Initialize edges array for 10 edges 
        edges = zeros(num_points, 2, 10);
        
        % Compute new "shattered" horizontal edges
        xedge57 = linspace(theta1(1), thobs_lim(1), num_points);
        xedge68 = linspace(thobs_lim(2), theta1(num_points), num_points);

        % Define the edges based on the joint limits and obstacle limits

        % Left vertical edge: theta1 is constant at theta1(1), theta2 varies
        edges(:, 1, 1) = theta1(1);
        edges(:, 2, 1) = theta2;

        % Right vertical edge: theta1 is constant at theta1(num_points), theta2 varies
        edges(:, 1, 2) = theta1(num_points);
        edges(:, 2, 2) = theta2;

        % Left vertical obstacle edge: theta1 is constant at thobs_lim(1), theta2 varies
        edges(:, 1, 3) = thobs_lim(1);          
        edges(:, 2, 3) = theta2;

        % Right vertical obstacle edge: theta1 is constant at thobs_lim(2), theta2 varies
        edges(:, 1, 4) = thobs_lim(2);
        edges(:, 2, 4) = theta2;

        % Top left horizontal edge: theta2 is constant at theta2(num_points), theta1 varies from theta1(1) to thobs_lim(1)
        edges(:, 1, 5) = xedge57;
        edges(:, 2, 5) = theta2(num_points);

        % Top right horizontal edge: theta2 is constant at theta2(num_points), theta1 varies from thobs_lim(2) to theta1(num_points)
        edges(:, 1, 6) = xedge68;
        edges(:, 2, 6) = theta2(num_points);

        % Bottom left horizontal edge: theta2 is constant at theta2(1), theta1 varies from theta1(1) to thobs_lim(1)
        edges(:, 1, 7) = xedge57;
        edges(:, 2, 7) = theta2(1);

        % Bottom right horizontal edge: theta2 is constant at theta2(1), theta1 varies from thobs_lim(2) to theta1(num_points)
        edges(:, 1, 8) = xedge68;
        edges(:, 2, 8) = theta2(1);

        % Middle left horizontal edge at theta2 = 0: theta1 varies from theta1(1) to thobs_lim(1), theta2 is constant at 0
        edges(:, 1, 9) = xedge57;
        edges(:, 2, 9) = 0;

        % Middle right horizontal edge at theta2 = 0: theta1 varies from thobs_lim(2) to theta1(num_points), theta2 is constant at 0
        edges(:, 1, 10) = xedge68;
        edges(:, 2, 10) = 0;                    
    end

end