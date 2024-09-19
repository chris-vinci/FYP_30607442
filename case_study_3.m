close all; clear; clc;
%% CREATING NOMINAL AND MIXED PLANTS

% define parameters for the inverted pendulum model transfer functions
% G_x(s) and G_phi(s)
M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

% define inverted pendulum model transfer functions G_x(s) and
% G_phi(s)
G_x = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);
G_x.InputName = 'u'; G_x.OutputName = 'x';
G_phi = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);
G_phi.InputName = 'u'; G_phi.OutputName = 'phi';

% check if they are passive systems
isPassive(G_x)
isPassive(G_phi)
% confirm with Nyquist plot
figure
nyquist(G_x)
hold on
plot(cos(linspace(0,2*pi,1000)),sin(linspace(0,2*pi,1000))) % unit circle
hold off

%% COMPUTING LQG CONTROLLERS FOR OUTPUT: x (G_x)

% compute optimal LQG controller for objective given in cs2 problem solution sheet, with noise variances
[a,b,c,d] = ssdata(G_x);
M = [c d;zeros(1,length(c)) 1]; % [y;u] = M * [x;u]
QWV = blkdiag(b*b',1e-2); % noise variance: d -> 1, n -> 0.01
QXU = M'*diag([1 1e-3])*M;
CLQG_x = lqg(ss(G_x),QXU,QWV); % state-space model required for lqg

% 2nd-order state-space controller
C = ltiblock.ss('C',2,1,1);
% closed-loop model
C.InputName = 'yn'; C.OutputName = 'u';
S1 = sumblk('yn = y + n');
S2 = sumblk('uG = u + d');
CL0 = connect(Gp,C,S1,S2,{'d','n'},{'y','u'},{'yn','u'});

% tuning goal for both controllers
R1 = TuningGoal.LQG({'d','n'},{'y','u'},diag([1,1e-2]),diag([1 1e-3])); % function coefficients inside integral of J: y -> 1, u -> 0.001

% constraint for passive controller
R2 = TuningGoal.WeightedPassivity({'yn'},{'u'},-1,1);
R2.Openings = 'u';

% constraints for mixed controller, frequency close to gain boundary
R2g = TuningGoal.Gain({'yn'},{'u'},1);
R2g.Focus = [1.02,Inf];
R2p = TuningGoal.WeightedPassivity({'yn'},{'u'},-1,1);
R2p.Openings = 'u';
R2p.Focus = [0,1.02];

% tune controllers
CL1p = systune(CL0,R1,R2); % passive controller
CL1m = systune(CL0,R1,[R2g,R2p]); % mixed controller

%% COMPUTING LQG CONTROLLERS FOR OUTPUT: phi (G_phi)

% compute optimal LQG controller for objective given in cs2 problem solution sheet, with noise variances
[a,b,c,d] = ssdata(G_phi);
M = [c d;zeros(1,length(c)) 1]; % [y;u] = M * [x;u]
QWV = blkdiag(b*b',1e-2); % noise variance: d -> 1, n -> 0.01
QXU = M'*diag([1 1e-3])*M;
CLQG_phi = lqg(ss(G_phi),QXU,QWV); % state-space model required for lqg

% 2nd-order state-space controller
C = ltiblock.ss('C',2,1,1);
% closed-loop model
C.InputName = 'yn'; C.OutputName = 'u';
S1 = sumblk('yn = y + n');
S2 = sumblk('uG = u + d');
CL0 = connect(Gp,C,S1,S2,{'d','n'},{'y','u'},{'yn','u'});

% tuning goal for both controllers
R1 = TuningGoal.LQG({'d','n'},{'y','u'},diag([1,1e-2]),diag([1 1e-3])); % function coefficients inside integral of J: y -> 1, u -> 0.001

% constraint for passive controller
R2 = TuningGoal.WeightedPassivity({'yn'},{'u'},-1,1);
R2.Openings = 'u';

% constraints for mixed controller, frequency close to gain boundary
R2g = TuningGoal.Gain({'yn'},{'u'},1);
R2g.Focus = [1.02,Inf];
R2p = TuningGoal.WeightedPassivity({'yn'},{'u'},-1,1);
R2p.Openings = 'u';
R2p.Focus = [0,1.02];

% tune controllers
CL1p = systune(CL0,R1,R2); % passive controller
CL1m = systune(CL0,R1,[R2g,R2p]); % mixed controller

%% IMPULSE RESPONSES

% naming convention for Txyz
% x = controller (0 - optimal, p - passive, m - mixed)
% y = plant (p - passive, m - mixed)
% z = input (d - process noise, n - measurement noise)

% FROM d TO y
% passive plant
T0pd = feedback(Gp,CLQG,+1);
% Tppd = feedback(Gp,getBlockValue(CL1p,'C'),+1);
Tppd = getIOTransfer(CL1p,'d','y');
% Tmpd = feedback(Gp,getBlockValue(CL1m,'C'),+1);
Tmpd = getIOTransfer(CL1m,'d','y');

% mixed plant
T0md = feedback(Gm,CLQG,+1);
Tpmd = feedback(Gm,getBlockValue(CL1p,'C'),+1);
Tmmd = feedback(Gm,getBlockValue(CL1m,'C'),+1);

% FROM n TO y
% passive plant
T0pn = feedback(Gp*CLQG,1,+1);
Tppn = feedback(Gp*getBlockValue(CL1p,'C'),1,+1);
Tmpn = feedback(Gp*getBlockValue(CL1m,'C'),1,+1);

% mixed plant
T0mn = feedback(Gm*CLQG,1,+1);
Tpmn = feedback(Gm*getBlockValue(CL1p,'C'),1,+1);
Tmmn = feedback(Gm*getBlockValue(CL1m,'C'),1,+1);

%% PLOTTING

figure
% impulse responses FROM d TO y
subplot(2,2,1)
impulse(T0pd,Tppd,Tmpd,5)
title('(a) Response to Impulse Disturbance d for Passive Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG')

subplot(2,2,2)
impulse(T0md,Tpmd,Tmmd,5)
title('(b) Response to Impulse Disturbance d for Mixed Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG')

% impulse responses FROM n TO y
subplot(2,2,3)
impulse(T0pn,Tppn,Tmpn,5)
title('(c) Response to Impulse Disturbance n for Passive Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG')

subplot(2,2,4)
impulse(T0mn,Tpmn,Tmmn,5)
title('(d) Response to Impulse Disturbance n for Mixed Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG')

%% STABILITY CHECKS

% initialising variables
controllers = {'T0pd','Tppd','Tmpd','T0md','Tpmd','Tmmd','T0pn','Tppn','Tmpn','T0mn','Tpmn','Tmmn'};
stability = zeros(1,length(controllers),'logical');

% check stability
for i = 1:length(controllers)
    current_controller = evalin('base',controllers{i});
    [~,p,~] = zpkdata(current_controller);
    stability(i) = all(real(p{1}) < 0);
end

stability_results = cell2struct(num2cell(stability),controllers,2);