close all; clear; clc;
%% CREATING NOMINAL (PASSIVE) AND REAL (MIXED) PLANTS

% define parameters for the beam model transfer function G(s)
xi = 0.05;
alpha = [0.09877 -0.309 -0.891 0.9 0.7071 -0.8091];
w = [1 4 9 16 25 36];
eps = [0.1 1.5 0.72 0.56 20 100];

% define passive beam model transfer function Gp(s)
Gp = tf(alpha(1)^2*[1 0],[1 2*xi*w(1) w(1)^2]);
for i = 2:6
    Gp = Gp + tf(alpha(i)^2*[1 0],[1 2*xi*w(i) w(i)^2]);
end
Gp.InputName = 'uG'; Gp.OutputName = 'y';
% check if the beam is a passive system
isPassive(Gp)
% confirm with Nyquist plot
figure
nyquist(Gp) % Gp is positive real on the Nyquist plot and is hence passive
hold on
plot(cos(linspace(0,2*pi,1000)),sin(linspace(0,2*pi,1000))) % unit circle
% frequency selected close to gain boundary: 1.02
hold off

% define mixed beam model transfer function Gm(s)
Gm = tf([alpha(1)^2 eps(1)],[1 2*xi*w(1) w(1)^2]);
for i = 2:6
    Gm = Gm + tf([alpha(i)^2 eps(i)],[1 2*xi*w(i) w(i)^2]);
end
Gm.InputName = 'uG'; Gm.OutputName = 'y';
% check if the beam is a passive system
isPassive(Gm)
% confirm with Nyquist plot
figure
nyquist(Gm) % Gm is mixed
hold on
plot(cos(linspace(0,2*pi,1000)),sin(linspace(0,2*pi,1000))) % unit circle
% frequency selected close to gain boundary: 1.02
hold off

%% COMPUTING LQG CONTROLLERS

% compute optimal LQG controller for objective given in problem solution sheet, with noise variances
[a,b,c,d] = ssdata(Gp);
M = [c d;zeros(1,length(c)) 1]; % [y;u] = M * [x;u]
QWV = blkdiag(b*b',1e-2); % noise variance: d -> 1, n -> 0.01
QXU = M'*diag([1 1e-3])*M;
CLQG = lqg(ss(Gp),QXU,QWV); % state-space model required for lqg

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

%% CLOSED-LOOP SYSTEMS

% naming convention for Txyz
% x = input (d - process noise, n - measurement noise)
% y = plant (p - passive, m - mixed)
% z = controller (0 - optimal, p - passive, m - mixed)

% FROM d TO y
% passive plant
Tdp0 = feedback(Gp,CLQG,+1);
% Tppd = feedback(Gp,getBlockValue(CL1p,'C'),+1);
Tdpp = getIOTransfer(CL1p,'d','y');
% Tmpd = feedback(Gp,getBlockValue(CL1m,'C'),+1);
Tdpm = getIOTransfer(CL1m,'d','y');

% mixed plant
Tdm0 = feedback(Gm,CLQG,+1);
Tdmp = feedback(Gm,getBlockValue(CL1p,'C'),+1);
Tdmm = feedback(Gm,getBlockValue(CL1m,'C'),+1);

% FROM n TO y
% passive plant
Tnp0 = feedback(Gp*CLQG,1,+1);
Tnpp = feedback(Gp*getBlockValue(CL1p,'C'),1,+1);
Tnpm = feedback(Gp*getBlockValue(CL1m,'C'),1,+1);

% mixed plant
Tnm0 = feedback(Gm*CLQG,1,+1);
Tnmp = feedback(Gm*getBlockValue(CL1p,'C'),1,+1);
Tnmm = feedback(Gm*getBlockValue(CL1m,'C'),1,+1);

%% PLOTTING
t_settle = 100;
t_transient = 10;

% plots to show settling behaviour
figure
% step responses FROM d TO y
subplot(2,2,1)
step(Tdp0,Tdpp,Tdpm,t_settle)
title('(a) Response to Step Disturbance d for Passive Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG')

subplot(2,2,2)
step(Tdm0,Tdmp,Tdmm,t_settle)
title('(b) Response to Step Disturbance d for Mixed Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG','Location','southeast')

% step responses FROM n TO y
subplot(2,2,3)
step(Tnp0,Tnpp,Tnpm,t_settle)
title('(c) Response to Step Disturbance n for Passive Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG','Location','southeast')

subplot(2,2,4)
step(Tnm0,Tnmp,Tnmm,t_settle)
title('(d) Response to Step Disturbance n for Mixed Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG','Location','southeast')

% plots to show transient behaviour
figure
% step responses from d TO y
subplot(2,2,1)
step(Tdp0,Tdpp,Tdpm,t_transient)
title('(a) Response to Step Disturbance d for Passive Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG')

subplot(2,2,2)
step(Tdm0,Tdmp,Tdmm,t_transient)
title('(b) Response to Step Disturbance d for Mixed Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG','Location','southeast')

% step responses FROM n TO y
subplot(2,2,3)
step(Tnp0,Tnpp,Tnpm,t_transient)
title('(c) Response to Step Disturbance n for Passive Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG','Location','southeast')

subplot(2,2,4)
step(Tnm0,Tnmp,Tnmm,t_transient)
title('(d) Response to Step Disturbance n for Mixed Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG','Location','southeast')

% plot for poster
figure
step(Tnm0,Tnmp,Tnmm,t_settle)
title('Response to Step Disturbance n for Mixed Plant')
legend('Optimal LQG','2nd-order passive LQG','2nd-order mixed LQG','Location','southeast')

%% STABILITY CHECKS

% initialising variables
closed_loop_systems = {'Tdp0','Tdpp','Tdpm','Tdm0','Tdmp','Tdmm','Tnp0','Tnpp','Tnpm','Tnm0','Tnmp','Tnmm'};
stability = zeros(1,length(closed_loop_systems),'logical');
max_real_pole = zeros(1,length(closed_loop_systems));

% check all closed-loop systems
for i = 1:length(closed_loop_systems)
    current_system = evalin('base',closed_loop_systems{i});
    [~,p,~] = zpkdata(current_system);
    poles = cell2mat(p);

    stability(i) = all(real(poles) < 0); % stability
    % max_real_pole(i) = max(real(poles)); % greatest pole real component
    [~,j] = max(real(poles));
    max_real_pole(i) = poles(j);
end

stability_results_table = table(closed_loop_systems',stability',max_real_pole',...
    'VariableNames',{'System','IsStable','MaxRealPole'});

disp(stability_results_table)
