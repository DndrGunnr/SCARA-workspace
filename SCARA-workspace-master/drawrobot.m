function [elbSta, robotArmsc, jrobotArmsc] = drawrobot( x, y, l1, l2, th1min, th1max, th2min, th2max, xo, yo, ro, fig, graph, graphjoint, textcom, robotArmso, jrobotArmso)  
    
    set(textcom,'Visible','off');
    %calculate joint parameters    
    thInB = zeros(2,2);
    %theta1
    e = [-1 1];
    b = (x^2+y^2+l1^2-l2^2)/(2*l1);
    
    if (y^2+x^2-b^2<0)
        set(fig,'CurrentAxes',graph);
        hold on
        set(textcom,'String','This point is not reachable','Visible','on');
        elbSta = [0 0];
        if nargin > 15
            if ishandle(robotArmso(1))
                delete(robotArmso(1));
            end
            if ishandle(robotArmso(2))
                delete(robotArmso(2));
            end
        end
        for i = 1:2
            robotArmsc(i) = plot(graph,[0 0 0 0 0],[0 0 0 0 0],'-k');
        end
        if nargin > 16
           delete(jrobotArmso);
        end
        jrobotArmsc = plot(graphjoint,[0 0],[0 0],'ok');
    else
        c1 = (x*b-e*y*sqrt(y^2+x^2-b^2))/(y^2+x^2);
        s1 = (y*b+e*x*sqrt(y^2+x^2-b^2))/(y^2+x^2);
        xt = l1*c1; yt = l1*s1;
        for i = 1:2
            if c1(i) == 0
                th(1,i) = pi/2*sign(s1(i));
            else
                if s1(i)>=0
                    th(1,i) = rem(acos(c1(i)),pi);
                else
                    th(1,i) = rem(-acos(c1(i)),pi);
                end
            end
        end
        c12 = (x-xt)/l2; s12 = (y-yt)/l2;
        for i = 1:2
            if c12(i) == 0
                if pi/2*sign(s12(i)) - th(1,i) > pi
                    th(2,i) = pi/2*sign(s12(i)) - th(1,i)-2*pi;
                elseif pi/2*sign(s12(i)) - th(1,i) < -pi
                    th(2,i) = pi/2*sign(s12(i)) - th(1,i)+2*pi;
                else
                    th(2,i) = pi/2*sign(s12(i)) - th(1,i);
                end
            else
                if c12(i)>=0
                    if asin(s12(i))-th(1,i) > pi
                        th(2,i) = asin(s12(i))-th(1,i)-2*pi;
                    elseif asin(s12(i))-th(1,i) < -pi
                        th(2,i) = asin(s12(i))-th(1,i)+2*pi;
                    else
                        th(2,i) = asin(s12(i))-th(1,i);
                    end
                else
                    if pi-asin(s12(i))-th(1,i) <= pi
                        th(2,i) = pi-asin(s12(i))-th(1,i);
                    else
                        th(2,i) = -pi-asin(s12(i))-th(1,i);
                    end
                end
            end
        end
        
        %Check if position is in bounds
        for i = 1:2
            if (th(1,i)>th1max*(pi/180) || th(1,i)<th1min*(pi/180))
                thInB(1,i) = 0;
            else
                thInB(1,i) = 1;
            end
        end
        for i = 1:2
            if (th(2,i)>th2max*(pi/180) || th(2,i)<th2min*(pi/180))
                thInB(2,i) = 0;
            else
                thInB(2,i) = 1;
            end
        end
        
        %elbow status
        for i = 1:2
            if (thInB(1,i)==1 && thInB(2,i)==1)      
                elbSta(i) = 1;
            else
                elbSta(i) = 0;
            end
        end
        
        % draw robot arms and specify elbow status
        set(fig,'CurrentAxes',graph);
        hold on
        if nargin > 15
            delete(robotArmso(1)); delete(robotArmso(2));
        end
        outB = 1;
        for i = 1:2
            if (thInB(1,i)==1 && thInB(2,i)==1)
                % check collision with obstacle
                r = testCol(th(1,i),th(2,i),l1,l2,xo,yo,ro);
                if  r == 0
                    robotArmsc(i) = plot(graph,[0 xt(i) x xt(i) 0],[0 yt(i) y yt(i) 0],'-k');
                elseif r == 1
                    set(textcom,'String','Collision with arm 1','Visible','on');
                    robotArmsc(i) = plot(graph,[0 xt(i) x xt(i) 0],[0 yt(i) y yt(i) 0],':r');
                elseif r == 2
                    set(textcom,'String','Collision with arm 2','Visible','on');
                    robotArmsc(i) = plot(graph,[0 xt(i) x xt(i) 0],[0 yt(i) y yt(i) 0],':r');
                elseif r == 3
                    set(textcom,'String','Collision with arm 1 & 2','Visible','on');
                    robotArmsc(i) = plot(graph,[0 xt(i) x xt(i) 0],[0 yt(i) y yt(i) 0],':r');
                end
                outB = 0;
            else
                robotArmsc(i) = plot(graph,[0 0 0 0 0],[0 0 0 0 0],'-k');
            end
        end
        if outB == 1
            set(textcom,'String','Point out of bounds','Visible','on');
        end            

        % on joint space
        set(fig,'CurrentAxes',graphjoint);
        hold on
        % draw joint point
        if nargin > 16
            delete(jrobotArmso);
        end
        jrobotArmsc = plot(graphjoint,(180/pi)*th(1,:),(180/pi)*th(2,:),'ok');
        xlim([-185 185]); ylim([-185 185]);
        xlabel('\theta_1'); ylabel('\theta_2');
        title('Joint space');
    end    
end