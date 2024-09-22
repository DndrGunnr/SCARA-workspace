function edges= edges_computation(theta1,theta2,thobs_lim)
%function to compute the edges of the joint space of the SCARA robot
%This function can be used for the allowable cases in the workspace
%computation:
% No obstacle
% obstacle that fits inside the joint space
% obstacle that partially goes out the outer joint space rectangle
    %resolution of theta range
    num_points = size(theta1,2);
    if nargin==2
        %this case covers both the no-obstacle case and the obstacle's
        %angles partially going beyond joint limits
        edges = zeros(num_points, 2, 5); % 5 bordi

        % Definisci i bordi basati sui limiti delle giunture
        edges(:, 1, 1) = theta1(1); % Bordo verticale sinistro
        edges(:, 2, 1) = theta2;          % Bordo sinistro variando theta2

        edges(:, 2, 2) = theta2(num_points);  % Bordo verticale destro
        edges(:, 1, 2) = theta1;          % Bordo destro variando theta1

        edges(:, 1, 3) = theta1(num_points);  % Bordo verticale destro
        edges(:, 2, 3) = theta2;          % Bordo destro variando theta2

        edges(:, 2, 4) = theta2(1);  % Bordo verticale sinistro
        edges(:, 1, 4) = theta1;          % Bordo sinistro variando theta1

        edges(:, 1, 5) = theta1;          % Bordo inferiore
        edges(:, 2, 5) = 0;                % Bordo inferiore a zero
    else
        %edges
        % |------5-----|////|-----6-------|
        %1|            |2//3|             |4
        % |------7-----|////|-----8-------|

        edges = zeros(num_points, 2, 9); % 8 bordi
        
        
        %new "shattered" horizontal edges
        xedge57=linspace(theta1(1),thobs_lim(1),num_points);
        xedge68=linspace(thobs_lim(2),theta1(num_points),num_points);


        
        %link 1 and obstacle
        edges(:, 1, 1) = theta1(1); % Bordo verticale sinistro
        edges(:, 2, 1) = theta2;

        edges(:, 1, 2) = theta1(num_points);  % Bordo verticale destro
        edges(:, 2, 2) = theta2;

        edges(:, 1, 3) = thobs_lim(1);  % Bordo verticale ostacolo sinistro
        edges(:, 2, 3) = theta2;

        edges(:, 1, 4) = thobs_lim(2);  % Bordo verticale ostacolo destro
        edges(:, 2, 4) = theta2;

        edges(:, 2, 5)= theta2(num_points); %bordo superiore sinistro
        edges(:, 1, 5)= xedge57;

        edges(:, 2, 6)= theta2(num_points); %bordo superiore sinistro
        edges(:, 1, 6)= xedge68;

        edges(:, 2, 7)= theta2(1); %bordo superiore sinistro
        edges(:, 1, 7)= xedge57;

        edges(:, 2, 8)= theta2(1); %bordo superiore sinistro
        edges(:, 1, 8)= xedge68;

        edges(:,2,9)=0;   %aspect change
        edges(:,1,9)=theta1;
    end

end

