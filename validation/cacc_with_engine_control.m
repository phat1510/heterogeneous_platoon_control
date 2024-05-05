% Author: Phat Do
% Apr 2024

% DESCRIPTION:

% 1. The application of structured H infinity control synthesis 
% with Matlab function 'hinfstruct' to solve the CACC problem

% 2. In this test case, the unstructure H infinity synthesis is applied 
% for acceleration control. The computed desired acceleration from CACC loop 
% is the input reference of the internal closed-loop system.

% Coupling protocol
% ui = Ke * ei + Kv * (vi - vj)

% Weighting functions
% v0 ---------> ei*We_e 
%             > aij(vi-vj)*We_v
%             > u_i*Wu

% Communication topology: PF

%% Reset the workspace
clc
clear 
close all;

%% Plants definition
h = 1.2; % !!! decrease time gap with reduce the platooning performance

A = [0 -1;0 0];
Bw = [1;0];
Bf1 = [-h;1]; B1 = [Bw Bf1];
Bf2 = [-h;1]; B2 = [Bw Bf2];
Bf3 = [-h;1]; B3 = [Bw Bf3];
C = [1 0;0 1];
D = zeros(2,2);
G1 = ss(A,B1,C,D); G1.u = {'v0','u1'}; G1.y = {'x1'}; % x1 = [e1 v1]
G2 = ss(A,B2,C,D); G2.u = {'x1(2)','u2'}; G2.y = {'x2'};
G3 = ss(A,B3,C,D); G3.u = {'x2(2)','u3'}; G3.y = {'x3'};

%% Engine closed-loop
s = tf('s');
tau = 0.4;
G_eng = 1/(tau*s+1);

%% Tunable gains
Ke_min = 0.2;
Ke_max = 1.1*Ke_min;

K11 = tunableGain('K11',1,1); K11.u = 'y1(1)'; K11.y = 'u11';
K11.Gain.Free = 1;
K11.Gain.Value = 0;
K11.Gain.Minimum = Ke_min;
K11.Gain.Maximum = Ke_max;
K12 = tunableGain('K12',0); K12.u = 'y1(2)'; K12.y = 'u12';
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
% Weighting function on ei
Ms=1; wb=1; epsi=1e-4;
We_e1=tf([1/Ms wb],[1 wb*epsi]);
We_e2=We_e1;
We_e3=We_e1;

% Weighting function on vi-vj
Ms=1;wb=0.5;epsi=0.001;
We_v=tf([1/Ms wb],[1 wb*epsi]);

% Weighting function on ui
Mu=2; wbc=1e5; epsi1=0.01; 
Wu0 = tf(1/Mu);
% Wu0=tf([1 wbc/Mu],[epsi1 wbc]); % It increases the hinfnorm(T)
 
% String stability check
Wz0 = tf(1);

% Define the regulated output
We1 = blkdiag(We_e1,We_v);We1.u = {'y1(1)','y1(2)'}; We1.y = 'z1';
We2 = blkdiag(We_e2,We_v);We2.u = {'y2(1)','y2(2)'}; We2.y = 'z2';
We3 = blkdiag(We_e3,We_v);We3.u = {'y3(1)','y3(2)'}; We3.y = 'z3';
Wu  = blkdiag(Wu0,Wu0,Wu0); Wu.u = {'u1','u2','u3'}; Wu.y = 'z4';
Wz  = blkdiag(Wz0,Wz0,Wz0); Wz.u = {'x1(2)','x2(2)','x3(2)'}; Wz.y = 'z5';

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
    Wu,...
    Wz,...
    K11,K12,K21,K22,K31,K32,...
    sum_y11,sum_y12,sum_y21,sum_y22,sum_y31,sum_y32,...
    sum_u1,sum_u2,sum_u3,...
    {'v0'},{'z1','z2','z3','z4','z5'})

%% Structured H infinity synthesis

% hinfstruct config
% rng('default')
rng('shuffle')
opt = hinfstructOptions('Display','final','RandomStart',0);

% Obtaint the closed-loop system with tunned parameters
[T,gamma,info] = hinfstruct(T0,opt);

% Extract
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

Ku1 = T(7,1)/Wu0;
Ku2 = T(8,1)/Wu0;
Ku3 = T(9,1)/Wu0;

% Plot
figure('Position',[500 500 1200 900])
% From v0 to ei-ej
subplot(221)
bodemag(w,1/We_e1,S11,S12,S13)
legend('1/We_e','S11','S12','S13');title('From v0 to (ei-ej)');grid on

% From v0 to vi-vj
subplot(222)
bodemag(w,1/We_v,S21,S22,S23)
legend('1/We_v','S21','S22','S23');title('From v0 to (vi-vj)');grid on

% From v0 to ue_i
subplot(223)
bodemag(w,1/Wu0,Ku1,Ku2,Ku3)
legend('1/Wu','Ku1','Ku2','Ku3');title('From v0 to u_i');grid on

% % From v0 to uv_i
% subplot(224)
% bodemag(w,1/Wu0,Kv1,Kv2,Kv3)
% legend('1/Wu','Kv1','Kv2','Kv3');title('From v0 to uv_i');grid on

% String stability check
% subplot(224)
% ST0_1 = T(10,1)/Wz0;
% ST0_2 = T(11,1)/Wz0;
% ST1_2 = ST0_2*inv(ST0_1);
% ST0_3 = T(12,1)/Wz0;
% ST2_3 = ST0_3*inv(ST0_2);
% bodemag(w,1/Wz0,ST0_1,ST1_2,ST2_3)
% legend('1/Wz','ST1','ST2','ST2');title('String stability check'),grid on


%% Simulation parameters
% Sampling time
Ts = 0.02;

% Simulation mode
sim_mode = 1; % standard velocity profile
% sim_mode = -1; % velocity with sin disturbance

% Standstill distance
lv = 4.5;
r = 8;

% Init spacing errors
p_init = [40 30 20 10];
e_init1 = (p_init(1)- p_init(2)-lv)-r;
e_init2 = (p_init(2)- p_init(3)-lv)-r;
e_init3 = (p_init(3)- p_init(4)-lv)-r;
e_init = [e_init1 e_init2 e_init3];

% Velocity profile params
v_ref0 = 0;
traction_accel = 0.5;
bracking_accel = -0.5;

dt = [30 15 30 10 20 5 10 15 0]'; % Define velocity periods
wp = tril(ones(length(dt)))*dt;
accel = [traction_accel 0 bracking_accel 0 bracking_accel 0 traction_accel 0]; % Corresponding acceleration


% Load models and designed controllers
load('models/G_leaf.mat'); 
load('models/G_prius.mat'); 
load('models/G_camry.mat');

load('controllers/K_leaf.mat'); 
load('controllers/K_prius.mat');
load('controllers/K_camry.mat');

% Engine models to be tested

test_case = 'case5';

switch test_case
    case 'case5'
    Ge_a_1 = G_leaf_a;
    Ge_a_2 = G_prius_a;
    Ge_a_3 = G_leaf_a;
    
    Ge_b_1 = G_leaf_b;
    Ge_b_2 = G_prius_b;
    Ge_b_3 = G_leaf_b;

    Ke_a_1 = K_leaf_a;
    Ke_a_2 = K_prius_a;
    Ke_a_3 = K_leaf_a;

    Ke_b_1 = K_leaf_b;
    Ke_b_2 = K_prius_b;
    Ke_b_3 = K_leaf_b;

end

%% Time response

sim("cacc_with_engine_control_sim.slx")

% Time
sim_time = tout;

% Spacing error
e_1 = spacing_error.Data(:,1);
e_2 = spacing_error.Data(:,2);
e_3 = spacing_error.Data(:,3);

% Spacing distance
d_0 = spacing_dis.Data(:,1);
d_1 = spacing_dis.Data(:,2);
d_2 = spacing_dis.Data(:,3);
d_3 = spacing_dis.Data(:,4);

% Velocity
v_0 = velocity.Data(:,1);
v_1 = velocity.Data(:,2);
v_2 = velocity.Data(:,3);
v_3 = velocity.Data(:,4);

% Accel
a_1 = acceleration.Data(:,1);
a_2 = acceleration.Data(:,2);
a_3 = acceleration.Data(:,3);
a_des = acceleration.Data(:,4);

% Velocity error
e_v1 = v_1-v_0;
e_v2 = v_2-v_1;
e_v3 = v_3-v_2;

%% Visualization
xlim_max =100;
xlim_min = 15;
lw = 1.0;

figure('Position', [500 500 1500 900])


% subplot(221)
% plot(sim_time, v_0, ...
%     sim_time, v_1, ...
%     sim_time, v_2, ...
%     sim_time, v_3, ...
%     LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-22 26])
% legend("car_0", "car_1", "car_2", "car_3");
% ylabel('v_i(m/s)')
% grid on
% 
% subplot(222)
% plot(sim_time, e_1, ...
%     sim_time, e_2, ...
%     sim_time, e_3, ...
%     LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-1 1])
% legend("car_1", "car_2", "car_3");
% xlabel('t(s)')
% ylabel('e_i(m)')
% grid on
% 
% subplot(223)
% plot(sim_time, d_1, ...
%     sim_time, d_2, ...
%     sim_time, d_3, ...
%     LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-15 15])
% legend("car_1", "car_2", "car_3");
% xlabel('t[s]')
% ylabel('d_i[m]')
% grid on
% 
% subplot(224)
% plot(sim_time, a_1, ...
%     sim_time, a_2, ...
%     sim_time, a_3, ...
%     LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-1.5 1.5])
% legend("a1", "a2", "a3");
% xlabel('t(s)')
% ylabel('a_i(m/s^2)')
% grid on

% 1 vehicles
% subplot(221)
% plot(sim_time, v_0, ...
%     sim_time, v_1, ...
%     LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-22 26])
% legend("v_{leader}", "v_1");
% xlabel('t[s]'); ylabel('v_i[m/s]')
% title('Velocity')
% grid on
% 
% subplot(222)
% plot(sim_time, e_1, ...
%     LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-1 1])
% legend("e_1");
% xlabel('t[s]');ylabel('e_i[m]')
% title('Spacing error')
% grid on
% 
% subplot(223)
% plot(sim_time, a_des, ...
%     sim_time, a_1, ...
%     LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-1.5 1.5])
% legend("a_{desired}","a_1");
% xlabel('t[s]'); ylabel('a_i[m/s^2]')
% title('Acceleration (open-loop)')
% grid on
% 
% subplot(224)
% plot(sim_time, d_0, ...
%     sim_time, d_1, ...
%     LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-15 15])
% legend("d_{desired}","d_1");
% xlabel('t[s]');ylabel('d_i[m]')
% title('Spacing distance')
% grid on

% 2 vehicles
subplot(221)
plot(sim_time, v_0, ...
    sim_time, v_1, ...
    sim_time, v_2, ...
    LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-22 26])
legend("v_{leader}", "v_{leaf}", "v_{prius}");
xlabel('t[s]'); ylabel('v_i[m/s]')
title('Velocity')
grid on

subplot(222)
plot(sim_time, e_1, ...
    sim_time, e_2, ...
    LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-1 1])
legend("e_1","e_2");
xlabel('t[s]');ylabel('e_i[m]')
title('Spacing error')
grid on

subplot(223)
plot(sim_time, a_1, ...
    sim_time, a_2, ...
    LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-1.5 1.5])
legend("a_1","a_2");
xlabel('t[s]'); ylabel('a_i[m/s^2]')
title('Acceleration (closed-loop)')
grid on

subplot(224)
plot(sim_time, d_0, ...
    sim_time, d_1, ...
    sim_time, d_2, ...
    LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-15 15])
legend("d_{desired}","d_1","d_2");
xlabel('t[s]');ylabel('d_i[m]')
title('Spacing distance')
grid on
