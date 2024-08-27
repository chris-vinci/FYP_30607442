close all; clear; clc;

% define parameters for the beam model transfer function G(s)
xi = 0.05;
% alpha = [0.09877 -0.309 -0.891 0.5878 0.7071 -0.8091];
alpha = [0.09877 -0.309 -0.891 0.9 0.7071 -0.8091];
w = [1 4 9 16 25 36];

% define beam model transfer function G(s)
G = tf(alpha(1)^2*[1 0],[1 2*xi*w(1) w(1)^2]);
for i = 2:6
    G = G + tf(alpha(i)^2*[1 0],[1 2*xi*w(i) w(i)^2]);
end
G.InputName = 'uG'; G.OutputName = 'y';
% check if the beam is a passive system
isPassive(G)
% confirm with Nyquist plot
nyquist(G) % G is positive real on the Nyquist plot and is hence passive

% compute optimal LQG controller for objective given in problem solution
% sheet, with noise variances
[a,b,c,d] = ssdata(G); % get state space data
M = [c d;zeros(1,12) 1]; % [y;u] = M * [x;u]
QWV = blkdiag(b*b',1e-2); % weighting matrix
QXU = M'*diag([1 1e-3])*M; % weighting matrix
[CLQG,info] = lqg(ss(G),QXU,QWV) % state-space model required for lqg
size(CLQG)
figure
bode(G,CLQG,{1e-2,1e3})
grid on
legend('G','CLQG')

% attempt to simplify controller using general-purpose tuner systune
% tune a 2nd-order state-space controller (can tune controllers of any
% order, not limited to full-order controller)
C = ltiblock.ss('C',2,1,1);
% closed-loop model of block diagram in Figure 2
C.InputName = 'yn'; C.OutputName = 'u';
S1 = sumblk('yn = y + n');
S2 = sumblk('uG = u + d');
CL0 = connect(G,C,S1,S2,{'d','n'},{'y','u'},{'yn','u'});
% use LQG criterion J (given in sheet) as sole tuning goal. allows direct
% specification of performance weights and noise covariances
R1 = TuningGoal.LQG({'d','n'},{'y','u'},diag([1 1e-2]),diag([1 1e-3]));
% 3rd input uses noise variances, 4th input uses coefficients found in J
% tune controller C to minimise LQG objective J
[CL1,J1] = systune(CL0,R1);
% optimiser found 2nd-order controller with J = 0.478
% compare with optimal J value for CLQG
[~,Jopt] = evalGoal(R1,replaceBlock(CL0,'C',CLQG))
100*(0.478-Jopt)/Jopt % percentage change
% performance degradation <5%, controller complexity reduced from 12 to 2
% states
% compare impulse responses from d to y for the two controllers
T0 = feedback(G,CLQG,+1);
T1 = getIOTransfer(CL1,'d','y');
figure
impulse(T0,T1,5)
title('Response to impulse disturbance d')
legend('LQG optimal','2nd-order LQG')
size(T0)
size(T1)
% subtract 12 states from both T0 and T1 gives 12 states for T0 and 2
% states for T1

% if -C(s) is passive, the closed-loop system will be stable
getPassiveIndex(-CLQG)
% the optimal CLQG controller is not passive. confirm with Nyquist plot
figure
nyquist(-CLQG) % clearly not passive

% can use systune to re-tune the 2nd-order controller along with the
% requirement that -C(s) should be passive. create passivity tuning goal
% for open-loop tf from yn to u (C(s)).
R2 = TuningGoal.WeightedPassivity({'yn'},{'u'},-1,1); 
% WeightedPassivity goal to account for minus sign
R2.Openings = 'u';

R3 = TuningGoal.Gain({'yn'},{'u'},1);
R3.Focus = [20,Inf];

% re-tune closed-loop model CL1 to minimise LQG objective J subject to
% -C(s) being passive
[CL2,J2,g] = systune(CL1,R1,[R2,R3]);
% tuner achieves same J value as before, but enforces passivity
% verify -C(s) is passive
C2 = getBlockValue(CL2,'C');
figure
passiveplot(-C2)
% more visible with Nyquist plot
figure
nyquist(-CLQG,-C2)
legend('LQG optimal','2nd-order passive LQG')
% compare impulse responses from d to y
T2 = getIOTransfer(CL2,'d','y');
figure
impulse(T0,T2,5)
title('Response to impulse disturbance d')
legend('LQG optimal','2nd-order passive LQG')
% systune has been used to design a second-order passive controller with
% near-optimal LQG performance
