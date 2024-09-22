function r = testCol(th1,th2,l1,l2,xo,yo,ro)
    
    % test collision between robot and obstacle
    % Warning : th in radians
    % r = -1 : The obstacle is always on the robot.
    % r = 0 : no collision
    % r = 1 : collision with arm 1
    % r = 2 : collision with arm 2
    % r = 3 : collision with arms 1 & 2
    
    if sqrt(xo^2+yo^2)-ro < 0
        r = -1;
    elseif l1+l2 < sqrt(xo^2+yo^2)-ro
        % obstacle is too far to collide the robot
        r = 0;
    else
        % test collision with arm 1
        xj = l1*cos(th1); yj = l1*sin(th1);
        dArm1O = (abs(yj*xo-xj*yo))/l1;    
        if dArm1O > ro 
            ri1 = 0;
        else
            dOH1 = (xo*xj+yo*yj)/l1; dOJ = sqrt(yj^2+xj^2);
            if (dOH1 < 0 && dOH1^2+dArm1O^2 > ro^2)
                ri1 = 0;
            elseif (dOH1 > l1 && (dOH1-dOJ)^2+dArm1O^2 > ro^2)
                ri1 = 0;
            else
                ri1 = 1;
            end
        end
        % test colision with arm 2
        x = xj + l2*cos(th1+th2); y = yj + l2*sin(th1+th2);
        dArm2O = (abs((y-yj)*xo+(xj-x)*yo+x*yj-y*xj))/l2;
        if dArm2O > ro 
            ri2 = 0;
        else
            dJH2 = ((xo-xj)*(x-xj)+(yo-yj)*(y-yj))/l2;
            dJX = sqrt((y-yj)^2+(x-xj)^2);
            if (dJH2 < 0 && dJH2^2+dArm2O^2 > ro^2)
                ri2 = 0;
            elseif (dJH2 > l2 && (dJH2-dJX)^2+dArm2O^2 > ro^2)
                ri2 = 0;
            else
                ri2 = 2;
            end
        end
        r = ri1+ri2;
    end
end