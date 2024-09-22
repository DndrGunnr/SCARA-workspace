function [th2omin, th2omax]=TestTh1(th1,l1,l2,xo,yo,ro,th2min,th2max)

% Test contacts with link2 when link1 is at th1
% If contact exists, returns th2omin and th2omax else return 999 and 999
% l1, l2= link lengths
% th1min, th1max, th2min, th2max = joint limits
% xo, yo= center of obstacle, ro=radius of disk obstacle
% only collisions with link2 considered
% geometric calculation
% xop, yop = disk obstacle center coordinates in robot's frame 2

c1=cos(th1);s1=sin(th1);
xop=c1*xo+s1*yo-l1;
yop=-s1*xo+c1*yo;
if sqrt(xop^2+yop^2)-ro>l2 %obstacle will never collide link2
    th2omin=999;th2omax=-999;
else
    rad=xop^2+yop^2-ro^2;
    if rad<=0 %obstacle contact by joint2
        th2omin=atan2(yop,xop)-pi/2;th2omax=atan2(yop,xop)+pi/2;
    else
        if l2^2>xop^2+yop^2-ro^2 %obstacle tangent contacts
            th2omin=atan2(yop,xop)-atan2(ro, sqrt(rad));
            th2omax=atan2(yop,xop)+atan2(ro, sqrt(rad));
        else %obstacle contact by link tip
            th2omin=atan2(yop,xop)-acos((xop^2+yop^2-ro^2+l2^2)/(2*l2*sqrt(xop^2+yop^2)));
            th2omax=atan2(yop,xop)+acos((xop^2+yop^2-ro^2+l2^2)/(2*l2*sqrt(xop^2+yop^2)));
        end;
    end;
    if (th2omax<th2min) || (th2omin>th2max)
        th2omin=999;th2omax=-999;
    end;
end;



