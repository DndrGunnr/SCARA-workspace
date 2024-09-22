function [xth2mi, yth2mi, xth2ma, yth2ma] = Bounds(th2mi,th2ma,th1,c1,s1,l1,l2)
%plot obst bounds for link2
s12 = sin(th1+th2mi);
c12 = cos(th1+th2mi);
xth2mi=l1*c1+l2*c12;
yth2mi=l1*s1+l2*s12;
hold on
s12 = sin(th1+th2ma);
c12 = cos(th1+th2ma);
xth2ma=l1*c1+l2*c12;
yth2ma=l1*s1+l2*s12;
hold on
