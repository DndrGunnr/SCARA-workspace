function calculatedraw( l1, l2, th1min, th1max, th2min, th2max, xo, yo, ro, fig, graph, graphjoint)
    
    % calculate workspace
    [T,P,S,ThoB] = Wscara(l1, l2, th1min, th1max, th2min, th2max, xo, yo, ro);
    
    set(fig,'CurrentAxes',graph);
    hold off
    % draw workspace
    plot(graph, T(:,1),T(:,2),'-g');
    hold on
    plot(graph, P(:,1),P(:,2),'.');
    % draw singularities
    plot(graph, S(:,1), S(:,2), '.c');
    % draw obstacle
    alpha=0:.1:2*pi;
    obstx=xo+ro*cos(alpha); obsty=yo+ro*sin(alpha);
    plot(obstx, obsty,'-r');
    legend('Robot in joint limit configurations','Workspace limits due to joint limits',...
        'Workspace limits due to singularities','Obstacle',...
        'Location','NorthOutside');
    title('Cartesian space');
    hold off
    axis equal;
    set(fig,'CurrentAxes',graphjoint);
    set(graphjoint,'XLim',[-185 185],'YLim',[-185 185]);
    hold off
    % draw singularities
    plot([-180 180 180 -180 -180 -180 180 180],[180 180 0 0 180 -180 -180 180],'-c');
    hold on
    % draw boundaries
    % check if there are collision with arm 1
    if ThoB(1,1) == (180/pi)*999 % there is no collision with arm 1
        plot(graphjoint,[th1min th1min th1max th1max th1min],[th2min th2max th2max th2min th2min],'b-');    
        % display annotations on graphs to specify the different areas.
        disptxt(graph,graphjoint,l1,l2,xo,yo,ro,th1min,th1max,th2min,th2max); 
    else       
        th1obstmin = ThoB(1,1);
        th1obstmax = ThoB(1,2);
        if th1obstmin < th1obstmax           
            if (th1obstmax < th1min || th1obstmin > th1max)
                plot(graphjoint,[th1min th1min th1max th1max th1min],[th2min th2max th2max th2min th2min],'b-');
            elseif (th1obstmax >= th1min && th1obstmin <= th1min && th1obstmax < th1max)
                plot(graphjoint,[th1obstmax th1obstmax th1max th1max th1obstmax],[th2min th2max th2max th2min th2min],'b-');
            elseif (th1obstmin <= th1max && th1obstmax >= th1max && th1obstmin > th1min)
                plot(graphjoint,[th1min th1min th1obstmin th1obstmin th1min],[th2min th2max th2max th2min th2min],'b-');
            elseif (th1obstmin >= th1min && th1obstmax <= th1max)
                 plot(graphjoint,[th1min th1min th1obstmin th1obstmin th1min],[th2min th2max th2max th2min th2min],'b-');
                 plot(graphjoint,[th1obstmax th1obstmax th1max th1max th1obstmax],[th2min th2max th2max th2min th2min],'b-');
            end
        else
            plot(graphjoint,[th1obstmax th1obstmax th1obstmin th1obstmin th1obstmax],[th2min th2max th2max th2min th2min],'b-');
        end
        
        if th1obstmin < th1obstmax       
            hatching(graphjoint,th1obstmin,th1obstmax);
        else
            hatching(graphjoint,-180,th1obstmax);
            hatching(graphjoint,th1obstmin,180);
        end
        % display annotations on graphs to specify the different areas.
        disptxt(graph,graphjoint,l1,l2,xo,yo,ro,th1min,th1max,th2min,th2max,th1obstmin,th1obstmax);
    end
    ThoB(1,:) = [];
    % drawing the shadow of the obstacle due to collisions with the arm 2
    plot(ThoB(:,1), ThoB(:,2),'.r','MarkerSize',3);
    xlim([-185 185]); ylim([-185 185]);
    xlabel('\theta_1'); ylabel('\theta_2');
    title('Joint space');
    axis equal
    set(graphjoint,'XLim',[-185 185],'YLim',[-185 185]);
    
end

function hatching(graphjoint,tmin,tmax)
    offset = 20;
    th1 = [];
    th2 = [];
    yi = -180;
    while (yi < 180-offset)
        th1 = [th1 tmin tmax tmax];
        th2 = [th2 yi yi+offset yi+2*offset];
        yi = yi+offset;
    end
    plot(graphjoint,th1,th2,'r-');
    plot(graphjoint,[tmin tmin],[-180 180],'r-');
    plot(graphjoint,[tmax tmax],[-180 180],'r-');
    hold on;
end
        

