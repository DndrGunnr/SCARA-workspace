function disptxt(graph,graphjoint,l1,l2,xo,yo,ro,th1min,th1max,th2min,th2max,th1obstmin,th1obstmax)
    % display annotations on graphs to specify the different areas.
    fig = gcf;
    
    % Annotations
    str1 = '1'; str2 = '2'; str3 = '3'; str4 = '4';
    % on joint space
    th = zeros(2,1);
    set(fig,'CurrentAxes',graphjoint);
    if nargin == 11 % th1obstmin and th1obstmax don't exist. There are no
                   % collision with arm 1.
       [th,st] = textTh1B(th2min,th2max,th1min,th1max,str1,str2,l1,l2,xo,yo,ro);       
    elseif nargin == 13
        if th1obstmin < th1obstmax         
            if (th1obstmax < th1min || th1obstmin > th1max)
                [th,st] = textTh1B(th2min,th2max,th1min,th1max,str1,str2,l1,l2,xo,yo,ro);
            elseif (th1obstmax >= th1min && th1obstmin <= th1min && th1obstmax < th1max)
                [th,st] = textTh1B(th2min,th2max,th1obstmax,th1max,str1,str2,l1,l2,xo,yo,ro);
            elseif (th1obstmin <= th1max && th1obstmax >= th1max && th1obstmin > th1min)
                [th,st] = textTh1B(th2min,th2max,th1min,th1obstmin,str1,str2,l1,l2,xo,yo,ro);
            elseif (th1obstmin >= th1min && th1obstmax <= th1max)
                [th,st] = textTh1B(th2min,th2max,th1min,th1obstmin,str1,str2,l1,l2,xo,yo,ro);
                [th2,st2] = textTh1B(th2min,th2max,th1obstmax,th1max,str3,str4,l1,l2,xo,yo,ro);
                th = horzcat(th,th2);
                st = [st st2];
            else
                return;
            end
        else
            [th,st] = textTh1B(th2min,th2max,th1obstmax,th1obstmin,str1,str2,l1,l2,xo,yo,ro);
        end
    end

    % on cartesian space
    set(fig,'CurrentAxes',graph);
    th = (pi/180)*th;
    for i = 1:length(th(1,:))
        x(i) = l1*cos(th(1,i))+l2*cos(th(1,i)+th(2,i));
        y(i) = l1*sin(th(1,i))+l2*sin(th(1,i)+th(2,i));
        text(x(i),y(i),st(i));
    end
       
    set(fig,'CurrentAxes',graphjoint);
end

function [th,str] = textTh1B(th2min,th2max,th1min,th1max,str1,str2,l1,l2,xo,yo,ro)
    text('HorizontalAlignment','center');
    if th2max > 0
        if th2min < 0
            th = bestPlace(th1min,th1max,0,th2max,l1,l2,xo,yo,ro)';
            ths = bestPlace(th1min,th1max,th2min,0,l1,l2,xo,yo,ro)';
            th = horzcat(th,ths);
            if ~isempty(th)
                str(1) = str1; 
                text(th(1,1),th(2,1),str1);
            end
            if length(th(1,:)) > 1
                str(2) = str2;
                text(th(1,2),th(2,2),str2);
            end
        else
            th(:,1) = bestPlace(th1min,th1max,th2min,th2max,l1,l2,xo,yo,ro);
            if ~isempty(th)
                str(1) = str1;
                text(th(1,1),th(2,1),str1);
            end
        end
    else
        th(:,1) = bestPlace(th1min,th1max,th2min,th2max,l1,l2,xo,yo,ro);
        if ~isempty(th)
            str(1) = str2;
            text(th(1,1),th(2,1),str2);
        end
    end
end

function thf = bestPlace(th1min,th1max,th2min,th2max,l1,l2,xo,yo,ro)
    % to avoid the obstacle
    cv = pi/180;
    th(1) = (th1max+th1min)/2; th(2) = (th2min+th2max)/2;
    if testCol(cv*th(1),cv*th(2),l1,l2,xo,yo,ro) > 0
        rcorner(1,1) = testCol(cv*th1min,cv*th2min,l1,l2,xo,yo,ro);               
        rcorner(2,1) = testCol(cv*th1min,cv*th2max,l1,l2,xo,yo,ro);
        rcorner(3,1) = testCol(cv*th1max,cv*th2max,l1,l2,xo,yo,ro);
        rcorner(4,1) = testCol(cv*th1max,cv*th2min,l1,l2,xo,yo,ro);
        rcorner(1,2) = th1min; rcorner(1,3) = th2min;               
        rcorner(2,2) = th1min; rcorner(2,3) = th2max;
        rcorner(3,2) = th1max; rcorner(3,3) = th2max;
        rcorner(4,2) = th1max; rcorner(4,2) = th2min;
        l = find(rcorner(:,1) == 0);
        thCoAv = rcorner(l,2:3);
        if ~(isempty(l))
            j = 1;
            r = 1;
            while (r > 0 && j<10)
                i = 1;
                while (i <= length(l) && r > 0)
                    th1c = (2*j-1)/(2*j)*thCoAv(i,1)+th(1)/(2*j);
                    th2c = (2*j-1)/(2*j)*thCoAv(i,2)+th(2)/(2*j);
                    r = testCol(cv*th1c,cv*th2c,l1,l2,xo,yo,ro);
                    i = i + 1;
                end
                j = j + 1;
            end
            if j < 10
                thf(1) = th1c; thf(2) = th2c;
            end
        end
    else
        thf(1) = th(1); thf(2) = th(2);
    end
end