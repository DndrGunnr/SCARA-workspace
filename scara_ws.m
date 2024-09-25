function scara_ws(th1_lim, th2_lim, L1, L2, x0, y0, r0)
    % Converti i limiti da gradi a radianti
    th1_lim_rad = deg2rad(th1_lim);
    th2_lim_rad = deg2rad(th2_lim);

    %resolution of joint "span"
    num_points=500;
    
    %span of joints
    theta2 = linspace(th2_lim_rad(1), th2_lim_rad(2), num_points);
    theta1 = linspace(th1_lim_rad(1), th1_lim_rad(2), num_points);


    %No obstacle case
    if nargin==4
        edges=edges_computation(theta1,theta2);

    %obstacle in the workspace
    else

        edges = zeros(num_points, 2, 9); % 8 bordi

        %TODO insert evaluation of collision between obstacle and link 2
        q2max=max(abs(th2_lim_rad(1)),abs(th2_lim_rad(2)));
        xee_min = L1 + L2 * cos(q2max);
        yee_min = L2 * sin(q2max);
        %minimum distance of EE from center
        radiusPlcm=sqrt(xee_min^2+yee_min^2);
        %distance of obstacle furthest point from center
        obsDist=sqrt(x0^2+y0^2)+r0;
        if(obsDist>radiusPlcm && obsDist<(L1+L2))
            error("obstacle may collide with link 2")
        end
        %compute radius of allowed area
        

        %computation of edges of joint space considering collision between
        %calcolo dei limiti introdotti dall'ostacolo
        theta_obs=atan2(y0,x0);
        %distanza tra origine e centro della circonferenza dell'ostacolo
        d=sqrt(y0^2+x0^2);
        alfa=asin(r0/d);
        thobs_lim=[theta_obs-alfa;theta_obs+alfa];
        rad2deg(thobs_lim)
    
        %considering the new boundary of the joint space as the
        %obstacle angle threshold
        if (obsDist>(L1+L2))
            edges= edges_computation(theta1,theta2);

        elseif thobs_lim(1)<th1_lim_rad(1)
            if thobs_lim(2)<th1_lim_rad(1)
                edges= edges_computation(theta1,theta2);
            else
                %the exluded joint space region overflows to the left
                theta1 = linspace(thobs_lim(2), th1_lim_rad(2), num_points);
                edges = edges_computation (theta1,theta2);
            end

        elseif thobs_lim(2)>th2_lim_rad(2)
            if thobs_lim(1)>th1_lim_rad(2)
                edges= edges_computation(theta1,theta2);
            else
                %the exluded joint space region overflows to the right
                theta1 = linspace(thobs_lim(2), thobs_lim(1), num_points);
                edges = edges_computation (theta1,theta2);
            end

        else
            %the exluded joint space region remains inside the "span" of the
            %first joint

            edges = edges_computation (theta1,theta2,thobs_lim);
        end

    end

    % Grafico dello spazio delle giunture in gradi
    figure;
    hold on;
    for i = 1:size(edges,3)
        plot(edges(:, 1, i) * (180/pi), edges(:, 2, i) * (180/pi), 'k', 'LineWidth', 1.5); % Contorni in nero
    end
    xlabel('Theta 1 (gradi)');
    ylabel('Theta 2 (gradi)');
    title('SCARA Joint Space');
    grid on;
    hold off;

    % Inizializza le matrici per le coordinate cartesiane
    X = [];
    Y = [];

    % Calcola le coordinate cartesiane per ogni combinazione di theta1 e theta2
    for i = 1:size(edges, 3)
        for j = 1:num_points
            % Equazioni di cinematica diretta
            X(j, i) = L1 * cos(edges(j, 1, i)) + L2 * cos(edges(j, 1, i) + edges(j, 2, i));
            Y(j, i) = L1 * sin(edges(j, 1, i)) + L2 * sin(edges(j, 1, i) + edges(j, 2, i));
        end
    end

    % Grafico dello spazio di lavoro cartesiano in radianti
    figure;
    hold on;
    plot(X, Y, 'k', 'LineWidth', 1.5); % Contorni in nero
    
    % Aggiungi ostacolo circolare
    theta_obs = linspace(0, 2*pi, 100); % Angoli per descrivere il cerchio
    x_circle = x0 + r0 * cos(theta_obs);
    y_circle = y0 + r0 * sin(theta_obs);
    plot(x_circle, y_circle, 'r--', 'LineWidth', 1.5); % Ostacolo in rosso
    
    % Etichette e titolo
    xlabel('X (m)');
    ylabel('Y (m)');
    title('SCARA Robot Workspace with Circular Obstacle');
    grid on;
    axis equal; % Scala uguale per gli assi X e Y
    hold off;

    % Stampa le dimensioni degli array
    disp(size(edges));
    disp(size(X));
end