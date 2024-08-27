close all; clear; clc;

% mixed system
m1 = tf(3,[1,3,2]);
nyquist(m1)
hold on
plot(cos(linspace(0,2*pi,1000)),sin(linspace(0,2*pi,1000)))
hold off

% not mixed system
m2 = tf(8,[1,4,3]);
figure
nyquist(m2)
hold on
plot(cos(linspace(0,2*pi,1000)),sin(linspace(0,2*pi,1000)))
hold off

%%
m3 = tf([20,0],[1,9,20]);
nyquist(m3)
hold on
plot(cos(linspace(0,2*pi,1000)),sin(linspace(0,2*pi,1000)))
hold off