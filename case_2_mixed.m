close all; clear; clc;

% define parameters for the beam model transfer function G(s)
xi = 0.05;
% alpha = [0.09877 -0.309 -0.891 0.5878 0.7071 -0.8091];
% alpha = [0.09877 -0.309 -0.891 0.5878 -0.7071 0.8091];
% w = [1 4 9 16 25 36];

% define beam model transfer function G(s)
% G = tf(alpha(1)^2*[1 0],[1 2*xi*w(1) w(1)^2]);
% for i = 2:6
%     G = G + tf(alpha(i)^2*[1 0],[1 2*xi*w(i) w(i)^2]);
% end

alpha = 0.8;
w = 4;
G = tf(alpha^2*[1 1],[1 2*xi*w w^2]);

G.InputName = 'uG'; G.OutputName = 'y';
% check if the beam is a passive system
isPassive(G)
% confirm with Nyquist plot
nyquist(G) % G is positive real on the Nyquist plot and is hence passive