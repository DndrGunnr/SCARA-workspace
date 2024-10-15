function [T,P,S,ThoB] = Scara2(l1,l2, th1min, th1max, th2min, th2max, xo, yo, ro)

%Called by main function Wscara
% Plots the workspace boundary of a 2-revolute jointed planar robot
% l1, l2= link lengths
% th1min, th1max, th2min, th2max = joint limits for which link 1 does not
% collide
% xo, yo= center of obstacle, ro=radius of disk obstacle
% collisions with link2 considered 
%
% External procedure called: 
%    SweepPlotTh1bounds
% External functions called: 
%  - TestTh1
%  - Bounds

T = zeros(1,2);
P = zeros(1,2);
S = zeros(1,2);

%Plot theta1 boundaries
th1bound(1)=th1min;th1bound(2)=th1max;
for i=1:2
    th1=th1bound(i);
    [th2omin, th2omax]=TestTh1(th1,l1,l2,xo,yo,ro,th2min,th2max);
    if th2omin==999 && th2omax==-999
        [Tint,Pint] = SweepPlotTh1bounds(th2min, th2max, th1,l1, l2);
        P = vertcat(P,Pint);
        T = vertcat(T,Tint);
    else
        if th2omin>th2min
            [Tint,Pint] = SweepPlotTh1bounds(th2min, th2omin, th1,l1, l2);
            P = vertcat(P,Pint);
            T = vertcat(T,Tint);
        end;
        if th2omax<th2max            
            [Tint,Pint] = SweepPlotTh1bounds(th2omax, th2max, th1,l1, l2);
            T = vertcat(T,Tint);
            P = vertcat(P,Pint);
        end;
    end;
end;

%Scan theta1 and plot theta2 obst boundaries
clear th1 th2 c1 s1 c12 s12
sing(1)=0;sing(2)=0;sing(3)=0;
step=0.01;

%preallocating vectors to save running time
xth2min=zeros(1,round(2*pi/step));yth2min=zeros(1,round(2*pi/step));
xth2max=zeros(1,round(2*pi/step));yth2max=zeros(1,round(2*pi/step));
xth2omin=zeros(1,round(2*pi/step));yth2omin=zeros(1,round(2*pi/step));
xth2omax=zeros(1,round(2*pi/step));yth2omax=zeros(1,round(2*pi/step));
xth2sing=zeros(3,round(2*pi/step));yth2sing=zeros(3,round(2*pi/step));

thsing(1)=0;thsing(2)=pi;thsing(3)=-pi;
i=1;
%To delete useless index in ThoB
indSup = [];
ThoB = zeros(2*floor((th1max-th1min)/step),2);
for th1=th1min:step:th1max;
    c1 = cos(th1);
    s1 = sin(th1);  
    [th2omin, th2omax] = TestTh1(th1,l1,l2,xo,yo,ro,th2min,th2max);
    if th2omin < 999
        ThoB(i,:) = [th1 th2omin];
    else
        indSup = [indSup,i];
    end
    if th2omax > -999
        ThoB(2*floor((th1max-th1min)/step)-i+1,:) = [th1 th2omax];
    else
        indSup = [indSup,2*floor((th1max-th1min)/step)-i+1];
    end
    if th2omin==999 && th2omax==-999
        [xth2min(i), yth2min(i), xth2max(i), yth2max(i)]=Bounds(th2min,th2max,th1,c1,s1,l1,l2);
    else
        if th2omin>th2min
            [xth2min(i), yth2min(i), xth2omin(i), yth2omin(i)]=Bounds(th2min,th2omin,th1,c1,s1,l1,l2);            
        end
        if th2omax<th2max
            [xth2omax(i), yth2omax(i), xth2max(i), yth2max(i)]=Bounds(th2omax,th2max,th1,c1,s1,l1,l2);          
        end
    end
    % build singularities if reachable
    for j=1:3
        if (th2omin>thsing(j) || th2omax<thsing(j))
            if (th2min<thsing(j) && th2max>thsing(j))
                sing(j)=1;
                th2=thsing(j);
                s12 = sin(th1+th2);
                c12 = cos(th1+th2);
                xth2sing(j,i)=l1*c1+l2*c12;
                yth2sing(j,i)=l1*s1+l2*s12;
            end
        end
    end
    i=i+1;
end

P = vertcat(P, [xth2min' yth2min'], [xth2max' yth2max'], [xth2omin' yth2omin'], [xth2omax' yth2omax']);

S = zeros(3*round(2*pi/step),2);
for j=1:3
    if sing(j)==1
        S = vertcat(S, [xth2sing(j,:)' yth2sing(j,:)']);
    end
end

%Delete useless index
ThoB(unique(indSup),:) = [];