 % Main file : EnveloppeRR.m
% Author : Quentin Pierron - Philippe Wenger
% February 2013 - october 2018
% Openq a window and draw the envelope of the workspace of a SCARA robot.
% The workspace depends on the joints limits th1min, th1max, th2min, th2max 
% on the obstacle position (x0,y0) and radius r0.

% initialization
close all
clear all
clc

%data
l1 = 1;
l2 = 1;
th1min = -132;
th1max = 132;
th2min = -141;
th2max = 141;
th1_limits=[th1min,th1max];
th2_limits=[th2min,th2max];
x0 = -0.25;
y0 = -0.25;
r0 = 0.1;

% display
affichage( l1, l2, th1min, th1max, th2min, th2max, x0, y0, r0);
scara_ws( th1_limits, th2_limits,l1, l2,x0, y0, r0);