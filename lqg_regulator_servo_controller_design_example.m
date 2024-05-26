close all; clear; clc;

% create state-space system
A = [0 1 0;0 0 1;1 0 0];    
B = [0.3 1;0 1;-0.3 0.9];
C = [1.9 1.3 1];  
D = [0.53 -0.61];
sys = ss(A,B,C,D);

% define noise covariance data and weighting matrices
nx = 3;    %Number of states
ny = 1;    %Number of outputs
Qn = [4 2 0; 2 1 0; 0 0 1];
Rn = 0.7;
R = [1 0;0 2];
QXU = blkdiag(0.1*eye(nx),R);
QWV = blkdiag(Qn,Rn);
QI = eye(ny);

% form LQG regulator
KLQG = lqg(sys,QXU,QWV)

% form one-dof LQG servo controller
KLQG1 = lqg(sys,QXU,QWV,QI,'1dof')

% form two-dof LQG servo controller
KLQG2 = lqg(sys,QXU,QWV,QI,'2dof')
