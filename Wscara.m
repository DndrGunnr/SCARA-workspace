function [T,P,S,ThoB] = Wscara(l1,l2, th1mind, th1maxd, th2mind, th2maxd, xo, yo, ro)

% Plots the workspace boundary of a 2-revolute jointed planar robot
% Robot has joint limits and a disk obstacle obstructs both links
% l1, l2= link lengths
% th1min, th1max, th2min, th2max = joint limits
% xo, yo= center of obstacle, ro=radius of disk obstacle
% it is assumed that xo^2+yo^2-ro^2>0 otherwise robot cannot be placed
%
% External procedure called: Scara2
%
textcom = findobj(gcf,'tag','tgtextcom');
if xo^2+yo^2-ro^2<=0
    set(textcom,'String','Robot always in collision','Visible','on');
    xo = 1; yo = 1; ro = 0.5;
    T = zeros(2,2); P = zeros(2,2); S = zeros(2,2); ThoB = zeros(2,2);
else
    set(textcom,'Visible','off');
    P = zeros(1,2);
    T = zeros(1,2);
    S = zeros(1,2);
    ThoB = zeros(1,2);
    ThoBint = zeros(1,2);
    %useless but don't clear
    P2 = zeros(1,2);
    T2 = zeros(1,2);
    S2 = zeros(1,2);
    ThoB2 = zeros(4,2);
    
    conv=pi/180;
    th1min=th1mind*conv;th2min=th2mind*conv;
    th1max=th1maxd*conv;th2max=th2maxd*conv;
    if sqrt(xo^2+yo^2)-ro>l1 %obstacle will never collide link1
        th1obstmin=999;th1obstmax=-999;
        ThoB(1,:) = (-1).*[th1obstmin th1obstmax]';
    else
        if l1^2>xo^2+yo^2-ro^2 %obstacle tangent contacts
            th1obstmin=atan2(yo,xo)-atan2(ro, sqrt(xo^2+yo^2-ro^2));
            th1obstmax=atan2(yo,xo)+atan2(ro, sqrt(xo^2+yo^2-ro^2));
        else %obstacle contact by link tip
            th1obstmin=atan2(yo,xo)-acos((xo^2+yo^2-ro^2+l1^2)/(2*l1*sqrt(xo^2+yo^2)));
            th1obstmax=atan2(yo,xo)+acos((xo^2+yo^2-ro^2+l1^2)/(2*l1*sqrt(xo^2+yo^2)));
        end;
        % "Normalisation"
%         if th1obstmin < -pi
%             th1obstmin = th1obstmin+2*pi;
%         end
%         if th1obstmax > pi
%             th1obstmax = th1obstmax-2*pi;
%         end
%         if th1obstmin>th1obstmax
%             th1om=th1obstmax;th1oM=th1obstmin;
%             th1obstmin=th1om;
%             th1obstmax=th1oM;
%         end;
    end;%obstacle will never collide link1
    
    ThoB(1,:) = [th1obstmin th1obstmax]';
    %if (th1obstmin > th1obstmax && th1obstmin < 999)
    %   [T,P,S,ThoB2] = Scara2(l1,l2, th1obstmax, th1obstmin, th2min, th2max, xo, yo, ro);
    %else
    if (th1obstmax<th1min || th1obstmin>th1max) || (th1obstmin<th1min && th1obstmax>th1max)
        [T,P,S,ThoB2] = Scara2(l1,l2, th1min, th1max, th2min, th2max, xo, yo, ro);
    else
        if (th1obstmin>th1min && th1obstmin<th1max)
            [T,P,S,ThoB2] = Scara2(l1,l2, th1min, th1obstmin, th2min, th2max, xo, yo, ro);
        end
        if (th1obstmax>th1min && th1obstmax<th1max)
            [Tint,Pint,Sint,ThoBint2] = Scara2(l1,l2, th1obstmax, th1max, th2min, th2max, xo, yo, ro);
            T = vertcat(T,Tint);
            P = vertcat(P,Pint);
            S = vertcat(S,Sint);
            ThoB2 = vertcat(ThoB2, ThoBint2);
        end
    end
    %end
    
    [T2,P2,S2,ThoBint] = Scara2(l1,l2, th1min, th1max, th2min, th2max, xo, yo, ro);
    ThoB = vertcat(ThoB,ThoBint);
    ThoB = (1/conv)*ThoB;
end;

