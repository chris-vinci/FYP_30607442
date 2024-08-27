close all; clear; clc;

% plant
G = tf(3,[1,3,2]);
G.InputName = 'uG';
G.OutputName = 'y';
nyquist(G)
hold on
plot(cos(linspace(0,2*pi,1000)),sin(linspace(0,2*pi,1000))) % unit circle
% frequency selected close to gain boundary: 0.945
% frequency selected close to passive boundary: 1.41
% frequency selected in middle of boundaries: 1.15
hold off
isPassive(G)
sys = ss(G);
[a,b,c,d] = ssdata(G);

% LQG controller
QWV = blkdiag(b*b',1e-2); % noise variance: d -> 1, n -> 0.01
M = [c,d;zeros(1,length(c)),1]; % [y;u] = M * [x;u]
QXU = M'*diag([1,1e-3])*M;
CLQG = lqg(sys,QXU,QWV);
tf_lqg = tf(CLQG)

% tune 2nd-order state-space controller
C = ltiblock.ss('C',2,1,1);
C.InputName = 'yn';
C.OutputName = 'u';
S1 = sumblk('yn = y + n');
S2 = sumblk('uG = u + d');
CL0 = connect(G,C,S1,S2,{'d','n'},{'y','u'},{'yn','u'});

% tuning goals
R1 = TuningGoal.LQG({'d','n'},{'y','u'},diag([1,1e-2]),diag([1 1e-3])); % function coefficients inside integral of J: y -> 1, u -> 0.001 

% CL1: frequency close to gain boundary
R2g = TuningGoal.Gain({'yn'},{'u'},1);
R2g.Focus = [0.945,Inf];
R2p = TuningGoal.WeightedPassivity({'yn'},{'u'},-1,1);
R2p.Openings = 'u';
R2p.Focus = [0,0.945];

% CL2: frequency close to passive boundary
R3g = TuningGoal.Gain({'yn'},{'u'},1);
R3g.Focus = [1.41,Inf];
R3p = TuningGoal.WeightedPassivity({'yn'},{'u'},-1,1);
R3p.Openings = 'u';
R3p.Focus = [0,1.41];

% CL3: frequency inbetween boundaries
R4g = TuningGoal.Gain({'yn'},{'u'},1);
R4g.Focus = [1.15,Inf];
R4p = TuningGoal.WeightedPassivity({'yn'},{'u'},-1,1);
R4p.Openings = 'u';
R4p.Focus = [0,1.15];

% tune controller C to minimise LQG objective J while following hard gain
% and passivity constraints
[CL1,J1] = systune(CL0,R1,[R2g,R2p]); % frequency close to gain boundary
[CL2,J2] = systune(CL0,R1,[R3g,R3p]); % frequency close to passive boundary
[CL3,J3] = systune(CL0,R1,[R4g,R4p]); % frequency inbetween boundaries

[CL4,J4] = systune(CL0,R1); % for comparison of soft and hard scores

% plotting impulse responses
T0 = feedback(G,CLQG,+1);
T1 = getIOTransfer(CL1,'d','y');
T2 = getIOTransfer(CL2,'d','y');
T3 = getIOTransfer(CL3,'d','y');

figure
subplot(2,2,1)
impulse(T0,T1)
title('(a) LQG optimal vs frequency near gain boundary')
legend('LQG optimal','$\Omega = 0.945','Interpreter','latex')

subplot(2,2,2)
impulse(T0,T2)
title('(b) LQG optimal vs frequency near passive boundary')
legend('LQG optimal','$\Omega = 1.41','Interpreter','latex')

subplot(2,2,3)
impulse(T0,T3)
title('(c) LQG optimal vs frequency inbetween boundaries')
legend('LQG optimal','$\Omega = 1.15','Interpreter','latex')

subplot(2,2,4)
impulse(T0,T1,T2,T3,5)
title('(d) Response to impulse disturbance d')
legend('LQG optimal','$\Omega = 0.945','$\Omega = 1.15','$\Omega = 1.41','Interpreter','latex')

% C1 = getBlockValue(CL1,'C');
% C2 = getBlockValue(CL2,'C');
% C3 = getBlockValue(CL3,'C');
% C4 = getBlockValue(CL4,'C');
% figure
% nyquist(-CLQG)
% system = ss(CLQG.A,CLQG.B,CLQG.C,CLQG.D);