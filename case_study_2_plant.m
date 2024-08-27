close all; clear; clc;

% define parameters for the beam model transfer function G(s)
xi = 0.05;
alpha = [0.09877 -0.309 -0.891 0.5878 0.7071 -0.8091];
w = [1 4 9 16 25 36];

% define beam model transfer function G(s)
% G = tf(alpha(1)^2*[1 0],[1 2*xi*w(1) w(1)^2]);
% for i = 2:6
%     G = G + tf(alpha(i)^2*[1 0],[1 2*xi*w(i) w(i)^2]);
% end

% eps = [9.76E-4 0.039 0.715 0.553 1.25 2.357];
eps = [0.001 0.04 0.72 0.56 1.3 2.4];
for i=1:6
    G = tf([alpha(i)^2 eps(i)],[1 2*xi*w(i) w(i)^2]);
    G.InputName = 'uG'; G.OutputName = 'y';
    isPassive(G)
    figure
    nyquist(G)
end

% G = tf([alpha(1)^2 eps(6)],[1 2*xi*w(6) w(6)^2]);
% G.InputName = 'uG'; G.OutputName = 'y';
% % check if the beam is a passive system
% isPassive(G)
% % confirm with Nyquist plot
% nyquist(G) % G is positive real on the Nyquist plot and is hence passive

%%
close all; clear; clc;

xi = 0.05;
alpha = [0.09877 -0.309 -0.891 0.5878 0.7071 -0.8091];
w = [1 4 9 16 25 36];

% eps = [9.76E-4 0.039 0.715 0.553 1.25 2.357];
for i=1:6
    G = tf(alpha(i)^2*[1 0],[1 2*xi*w(i) w(i)^2]);
    G.InputName = 'uG'; G.OutputName = 'y';
    isPassive(G)
    figure
    nyquist(G)
end