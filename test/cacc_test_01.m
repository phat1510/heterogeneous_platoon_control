% File name: cacc_test_01.m
% Author: Phat Do
% Apr 2024
% Description: The application of structured H infinity control synthesis 
% with Matlab function 'hinfstruct' to solve the CACC problem
% Case 1: ei*We_e 
%       : aij(vi-vj)*We_v
%       : ue_i*Wu_e + uv_i*Wu_v

%% Reset the workspace
clc
clear 
close all;

%% Plants
% m1 = 1500; m2 = 2000; m3 = 2500; %kg
m1 = 1460; m2 = 1460; m3 = 1460; %kg
h = 0.8;

A = [0 -1;0 0];
Bw = [1;0]; 
Bf1 = 1/m1*[-h;1]; B1 = [Bw Bf1];
Bf2 = 1/m2*[-h;1]; B2 = [Bw Bf2];
Bf3 = 1/m3*[-h;1]; B3 = [Bw Bf3];
C = [1 0;0 1];
D = zeros(2,2);
G1 = ss(A,B1,C,D); G1.u = {'v0','u1'}; G1.y = {'x1'}; % x1 = [e1 v1]
G2 = ss(A,B2,C,D); G2.u = {'x1(2)','u2'}; G2.y = {'x2'};
G3 = ss(A,B3,C,D); G3.u = {'x2(2)','u3'}; G3.y = {'x3'};

%% Tunable gains
Ke_max = 500;
Ke_min = 300;

K11 = tunableGain('K11',1,1); K11.u = 'y1(1)'; K11.y = 'u11';
K11.Gain.Free = 1;
K11.Gain.Value = 0;
K11.Gain.Minimum = Ke_min;
K11.Gain.Maximum = Ke_max;
K12 = tunableGain('K12',0); K12.u = 'y1(2)'; K12.y = 'u12';
% K12.Gain.Free = 1;
% K12.Gain.Value = 0;
% K12.Gain.Minimum = -1000;
% K12.Gain.Maximum = 0;
sum_u1 = sumblk('u1=u11+u12');

K21 = tunableGain('K21',0); K21.u = 'y2(1)'; K21.y = 'u21';
K21.Gain.Free = 1;
K21.Gain.Value = 0;
K21.Gain.Minimum = Ke_min;
K21.Gain.Maximum = Ke_max;
K22 = tunableGain('K22',0); K22.u = 'y2(2)'; K22.y = 'u22';
sum_u2 = sumblk('u2=u21+u22');

K31 = tunableGain('K31',0); K31.u = 'y3(1)'; K31.y = 'u31';
K31.Gain.Free = 1;
K31.Gain.Value = 0;
K31.Gain.Minimum = Ke_min;
K31.Gain.Maximum = Ke_max;
K32 = tunableGain('K32',0); K32.u = 'y3(2)'; K32.y = 'u32';
sum_u3 = sumblk('u3=u31+u32');

%% Desired performance by weighting functions
% Design of weighting function on ei
% Ms=1;wb=5;epsi=1e-4;
We_param = [1 1 1e-4];
We_e1=tf([1/We_param(1) We_param(2)],[1 We_param(2)*We_param(3)]);
We_e2=We_e1;
We_e3=We_e1;

% Weighting function on vi-vj
Ms=1;wb=0.5;epsi=0.001;
We_v=tf([1/Ms wb],[1 wb*epsi]);

We1 = blkdiag(We_e1,We_v);
We1.u = {'y1(1)','y1(2)'};
We1.y = 'z1';

We2 = blkdiag(We_e2,We_v);
We2.u = {'y2(1)','y2(2)'};
We2.y = 'z2';

We3 = blkdiag(We_e3,We_v);
We3.u = {'y3(1)','y3(2)'};
We3.y = 'z3';

% Weighting function on ui
Mu=1e2;wbc=10000;epsi1=0.01;
% Wu0=tf([1 wbc/Mu],[epsi1 wbc]);
Mu1=1e2;Mu2=1e2;Mu3=1e2;
Wu1 = tf(1/Mu1);
Wu2 = tf(1/Mu2);
Wu3 = tf(1/Mu3);
Wu_e = blkdiag(Wu1,Wu2,Wu3);
Wu_e.u = {'u11','u21','u31'}; Wu_e.y = 'z4';

Mu=3000;wbc=10000;epsi1=0.01;
% Wu0=tf([1 wbc/Mu],[epsi1 wbc]);
Wu0 = tf(1/Mu);
Wu_v = blkdiag(Wu0,Wu0,Wu0);
Wu_v.u = {'u12','u22','u32'}; Wu_v.y = 'z5';

% String stability check
Wz0 = tf(1);
Wz = blkdiag(Wz0,Wz0,Wz0); 
Wz.u = {'x1(2)', 'x2(2)', 'x3(2)'}; Wz.y = 'z6';

%% SUM blocks
sum_y11 = sumblk('y1(1) = x1(1)');
sum_y12 = sumblk('y1(2) = x1(2)-v0');

sum_y21 = sumblk('y2(1) = x2(1)');
sum_y22 = sumblk('y2(2) = x2(2)-x1(2)');

sum_y31 = sumblk('y3(1) = x3(1)');
sum_y32 = sumblk('y3(2) = x3(2)-x2(2)');

%% Generate the interconnected system

% Define the closed-loop system with tunable parameters
T0 = connect(G1,G2,G3,...
    We1,We2,We3,...
    Wu_e,Wu_v,...
    Wz,...
    K11,K12,K21,K22,K31,K32,...
    sum_y11,sum_y12,sum_y21,sum_y22,sum_y31,sum_y32,...
    sum_u1,sum_u2,sum_u3,...
    {'v0'},{'z1','z2','z3','z4','z5','z6'})

%% Structured H infinity synthesis

% hinfstruct config
% rng('default')
rng('shuffle')
opt = hinfstructOptions('Display','final','RandomStart',10);

% Obtaint the closed-loop system with tunned parameters
[T,gamma,info] = hinfstruct(T0,opt);

% Extrac
K11 = getBlockValue(T,'K11');
K21 = getBlockValue(T,'K21');
K31 = getBlockValue(T,'K31');

K12 = getBlockValue(T,'K12');
K22 = getBlockValue(T,'K22');
K32 = getBlockValue(T,'K32');

K1 = [K11 K12]
K2 = [K21 K22]
K3 = [K31 K32]

%% Data analysis
% Extract data
w = logspace(-4,3,100);
S11 = T(1,1)/We_e1;
S12 = T(3,1)/We_e2;
S13 = T(5,1)/We_e3;
[sv1,~] = sigma(1/We_e1,w);
[sv2,~] = sigma(S11,w);
[sv3,~] = sigma(S12,w);
[sv4,~] = sigma(S13,w);
v0_to_delta_e = [w; 20*log10(sv1); 20*log10(sv2); 20*log10(sv3); 20*log10(sv4)];

S21 = T(2,1)/We_v;
S22 = T(4,1)/We_v;
S23 = T(6,1)/We_v;
[sv1,~] = sigma(1/We_v,w);
[sv2,~] = sigma(S21,w);
[sv3,~] = sigma(S22,w);
[sv4,~] = sigma(S23,w);
v0_to_delta_v = [w; 20*log10(sv1) ;20*log10(sv2) ;20*log10(sv3); 20*log10(sv4)];

Ke1 = T(7,1)/Wu1;
Ke2 = T(8,1)/Wu2;
Ke3 = T(9,1)/Wu3;

Kv1 = T(10,1)/Wu0;
Kv2 = T(11,1)/Wu0;
Kv3 = T(12,1)/Wu0;

% fig_loc = '/home/phatdo/01_master_mars/01_Master_Thesis/master_report_cacc/dat/';
% fig_name = 'v0_to_delta_e_PF.txt';
% save(append(fig_loc, fig_name)); % Create a new .txt file
% fileID = fopen(append(fig_loc, fig_name),"w");
% fprintf(fileID,' %s %s %s %s %s\n','w','1/We','v1','v2','v3');
% fprintf(fileID,' %e %e %e %e %e\r\n',v0_to_delta_e);
% fclose(fileID);
% 
% fig_name = 'v0_to_delta_v_PF.txt'; % Create a new .txt file
% save(append(fig_loc, fig_name));
% fileID = fopen(append(fig_loc, fig_name),"w");
% fprintf(fileID,' %s %s %s %s %s\n','w','1/Wv','v1','v2','v3');
% fprintf(fileID,' %e %e %e %e %e\r\n',v0_to_delta_v);
% fclose(fileID);

% Plot
figure('Position',[500 500 1200 900])
% From v0 to ei-ej
subplot(221)
bodemag(w,1/We_e1,1/We_e2,1/We_e3,S11,S12,S13)
legend('1/We_e','S11','S12','S13');title('From v0 to ei-ej');grid on

% From v0 to vi-vj
subplot(222)
bodemag(w,1/We_v,S21,S22,S23)
legend('1/We_v','S21','S22','S23');title('From v0 to vi-vj');grid on

% From v0 to ue_i
subplot(223)
bodemag(w,1/Wu1,1/Wu2,1/Wu3,Ke1,Ke2,Ke3)
legend('1/Wu1','1/Wu2','1/Wu3','Ke1','Ke2','Ke3');title('From v0 to ue_i');grid on

% From v0 to uv_i
subplot(224)
bodemag(w,1/Wu0,Kv1,Kv2,Kv3)
legend('1/Wu','Kv1','Kv2','Kv3');title('From v0 to uv_i');grid on

% String stability check
% subplot(224)
% ST0_1 = T(10,1)/Wz0;
% ST0_2 = T(11,1)/Wz0;
% ST1_2 = ST0_2*inv(ST0_1);
% ST0_3 = T(12,1)/Wz0;
% ST2_3 = ST0_3*inv(ST0_2);
% bodemag(w,1/Wz0,ST0_1,ST1_2,ST2_3)
% legend('1/Wz','ST1','ST2','ST2');title('String stability check'),grid on

%% Config velocity profile
% Velocity profile params
v_ref0 = 0;
% Define velocity periods
dt = [10 5 20 10 20 5 10 15 0]';
wp = tril(ones(length(dt)))*dt;
accel = [1 0 1 0 -1 0 -0.5 0];

%% Time response

% Sim params
e_init = [2 -1 -2];

sim("cacc_test_01_sim.slx")
% 
sim_time = tout;

% Spacing error
e_1 = spacing_error.Data(:,1);
e_2 = spacing_error.Data(:,2);
e_3 = spacing_error.Data(:,3);

% Velocity
v_0 = velocity.Data(:,1);
v_1 = velocity.Data(:,2);
v_2 = velocity.Data(:,3);
v_3 = velocity.Data(:,4);

% Accel
a_1 = acceleration.Data(:,1);
a_2 = acceleration.Data(:,2);
a_3 = acceleration.Data(:,3);

lw = 1.0;

% Export data
% Time-response
% v_plot = velocity.Data(:,:);
% v_plot = [sim_time v_plot];
% fig_loc = '/home/phatdo/01_master_mars/01_Master_Thesis/master_report_cacc/dat/';
% fig_name = 'cacc_test_01_vel.dat';
% save(append(fig_loc, fig_name),'v_plot',"-ascii");
% e_plot = spacing_error.Data(:,:);
% e_plot = [sim_time e_plot];
% fig_name = 'cacc_test_01_err.dat';
% save(append(fig_loc, fig_name),'e_plot',"-ascii");

%% Visualization
figure('Position', [500 500 1500 900])
subplot(221)
plot(sim_time, v_0, ...
    sim_time, v_1, ...
    sim_time, v_2, ...
    sim_time, v_3, ...
    LineWidth=lw); xlim([0 80]); %ylim([-22 26])
% legend("car_0", "car_1", "car_2", "car_3");
ylabel('v_i(m/s)')
grid on

subplot(223)
plot(sim_time, a_1, ...
    sim_time, a_2, ...
    sim_time, a_3, ...
    LineWidth=lw); xlim([0 80]); %ylim([-15 15])
legend("a1", "a2", "a3");
xlabel('t(s)')
ylabel('a_i(m/s^2)')
grid on

subplot(222)
plot(sim_time, e_1, ...
    sim_time, e_2, ...
    sim_time, e_3, ...
    LineWidth=lw); xlim([0 80]); %ylim([-15 15])
legend("e1", "e2", "e3");
xlabel('t(s)')
ylabel('e_i(m)')
grid on
