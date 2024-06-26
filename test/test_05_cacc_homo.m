% Author: Phat Do
% Apr 2024

% DESCRIPTION:

% 1. The application of structured H-infinity control synthesis 
% with Matlab function 'hinfstruct' to solve the CACC problem

% 2. CACC mode
% \dot{e}_i(t) &= v_{i-1}(t) - v_{i}(t) - h_i u_i(t)
% \dot{v}_i(t) &= u_i(t)
% where u_i is the actual acceleration generated by engine

% 3. Platoon config
% ** 2PF topology
% ** 3 vehicles 

% 4. Coupling protocol
% ui = Ke * ei + Kv1 * (vi - vi-1) + Kv2 * (vi - vi-2)

% 5. Weighting functions
% v0 ---------> ei*We_e 
%             > aij(vi-vj)*We_v
%             > u_i*Wu
% v_{i-1}-----> v_i (string stability)

% 6. Tunable parameters: Ke, Kv1, Kv2

%% Reset the workspace
clc
clear 
close all;

%% Plants
h = 0.8;
A = [0 -1;0 0];
Bw = [1;0];
Bf1 = [-h;1]; B1 = [Bw Bf1];
Bf2 = [-h;1]; B2 = [Bw Bf2];
Bf3 = [-h;1]; B3 = [Bw Bf3];
C = [1 0;0 1];
D = zeros(2,2);
G1 = ss(A,B1,C,D); G1.u = {'v0','ueng1'}; G1.y = {'x1'}; % x1 = [e1 v1]
G2 = ss(A,B2,C,D); G2.u = {'x1(2)','ueng2'}; G2.y = {'x2'};
G3 = ss(A,B3,C,D); G3.u = {'x2(2)','ueng3'}; G3.y = {'x3'};

% Load models and designed controllers
load('models/G_leaf.mat'); 
load('models/G_prius.mat'); 
load('models/G_camry.mat');

% Engine models to be tested
Ge_a_1 = G_leaf_a;
Ge_a_2 = G_leaf_a;
Ge_a_3 = G_leaf_a;

% Ge_b_1 = G_leaf_a;
% Ge_b_2 = G_leaf_a;
% Ge_b_3 = G_leaf_a;

Ge_b_1 = G_leaf_b;
Ge_b_2 = G_leaf_b;
Ge_b_3 = G_leaf_b;

% Ge1 = G_leaf_a_2nd; 
% Ge1.u = {'u1'}; Ge1.y = {'ueng1'};
% Ge2 = G_leaf_a_2nd; 
% Ge2.u = {'u2'}; Ge2.y = {'ueng2'};
% Ge3 = G_leaf_a_2nd;
% Ge3.u = {'u3'}; Ge3.y = {'ueng3'};

Ge1 = G_leaf_a_2nd; 
Ge1.u = {'u1'}; Ge1.y = {'ueng1'};
Ge2 = G_leaf_a_2nd; 
Ge2.u = {'u2'}; Ge2.y = {'ueng2'};
Ge3 = G_leaf_a_2nd;
Ge3.u = {'u3'}; Ge3.y = {'ueng3'};

% s = tf('s');
% Ge = 1/(0.3*s+1);
% Ge1 = Ge; 
% Ge1.u = {'u1'}; Ge1.y = {'ueng1'};
% Ge2 = Ge; 
% Ge2.u = {'u2'}; Ge2.y = {'ueng2'};
% Ge3 = Ge; 
% Ge3.u = {'u3'}; Ge3.y = {'ueng3'};

%% SUM blocks
% Vehicle 1
sum_e1 = sumblk('y1(1) = x1(1)');
sum_v10 = sumblk('y1(2) = x1(2)-v0');

% Vehicle 2
sum_e2 = sumblk('y2(1) = x2(1)');
sum_v21 = sumblk('y21 = x2(2)-x1(2)');
sum_v20 = sumblk('y20 = x2(2)-v0');
% sum_v2   = sumblk('y2(2)=y21+y23');

% Vehicle 3
sum_e3 = sumblk('y3(1) = x3(1)');
sum_v32 = sumblk('y32 = x3(2)-x2(2)');
sum_y31 = sumblk('y31 = x3(2)-x1(2)');
% sym_y3 = sumblk('y3(2) = y32+y33');

%% Tunable gains
Ke_max = 0.2;
Ke_min = 0.99*Ke_max;

Kv_max = -0.5;
Kv_min = 1.1*Kv_max;

% Vehicle 1
Ke1 = tunableGain('Ke1',1,1); Ke1.u = 'y1(1)'; Ke1.y = 'ue1';
Ke1.Gain.Free = 1; Ke1.Gain.Value = 0;
Ke1.Gain.Minimum = Ke_min; Ke1.Gain.Maximum = Ke_max;
Kv1 = tunableGain('Kv1',0); Kv1.u = 'y1(2)'; Kv1.y = 'uv1';
Kv1.Gain.Free = 1;
Kv1.Gain.Value = 0;
Kv1.Gain.Minimum =  -1.01*1/h; Kv1.Gain.Maximum = -1/h;
% Kv1.Gain.Minimum =  Kv_min; Kv1.Gain.Maximum = Kv_max;
sum_u1 = sumblk('u1=ue1+uv1');

% Vehicle 2
Ke2 = tunableGain('Ke2',0); Ke2.u = 'y2(1)'; Ke2.y = 'ue2';
Ke2.Gain.Free = 1; Ke2.Gain.Value = 0;
Ke2.Gain.Minimum = Ke_min; Ke2.Gain.Maximum = Ke_max;
Kv21 = tunableGain('Kv21',0); Kv21.u = 'y21'; Kv21.y = 'uv21';
Kv21.Gain.Free = 1;
Kv21.Gain.Value = 0;
Kv21.Gain.Minimum = Kv_min; Kv21.Gain.Maximum = Kv_max;

Kv20 = tunableGain('Kv20',0); Kv20.u = 'y20'; Kv20.y = 'uv20';
Kv20.Gain.Free = 1; Kv20.Gain.Value = 0;
sum_u2 = sumblk('u2=ue2+uv21+uv20');

% Vehicle 3
Ke3 = tunableGain('Ke3',0); Ke3.u = 'y3(1)'; Ke3.y = 'ue3';
Ke3.Gain.Free = 1; Ke3.Gain.Value = 0;
Ke3.Gain.Minimum = Ke_min; Ke3.Gain.Maximum = Ke_max;
Kv32 = tunableGain('Kv32',0); Kv32.u = 'y32'; Kv32.y = 'uv32';
Kv32.Gain.Free = 1; Kv32.Gain.Value = 0;
Kv32.Gain.Minimum = Kv_min; Kv32.Gain.Maximum = Kv_max;


Kv31 = tunableGain('Kv31',0); Kv31.u = 'y31'; Kv31.y = 'uv31';
Kv31.Gain.Free = 1; Kv31.Gain.Value = 0;
sum_u3 = sumblk('u3=ue3+uv32+uv31');


%% Desired performance by weighting functions
% Weighting function on ei

Ms=1; wb=1; epsi=0.001; % how fast the vehicle 1 closes the gap
We_e1=tf([1/Ms wb],[1 wb*epsi]);

Ms=1; wb=3; epsi=0.001; % how fast the vehicle 2 closes the gap
We_e2=tf([1/Ms wb],[1 wb*epsi]);

Ms=1; wb=3; epsi=0.001; % how fast the vehicle 3 closes the gap
We_e3=tf([1/Ms wb],[1 wb*epsi]);

% Weighting function on vi-vj

% --On the first predecessor
We_v1 = ss(1);

% --On the second predecessor
We_v2 = ss(1);

% Weighting function on ui
Mu=2;
Wu0 = tf(1/Mu);
 
% String stability check
Wz0 = tf(1);

% Define the regulated output
We1 = blkdiag(We_e1,We_v1); We1.u = {'y1(1)','y1(2)'}; We1.y = 'z1';
We2 = blkdiag(We_e2,We_v1,We_v2); We2.u = {'y2(1)','y21','y20'}; We2.y = 'z2';
We3 = blkdiag(We_e3,We_v1,We_v2); We3.u = {'y3(1)','y32','y31'}; We3.y = 'z3';
Wu  = blkdiag(Wu0,Wu0,Wu0); Wu.u = {'u1','u2','u3'}; Wu.y = 'z4';
Wz  = blkdiag(Wz0,Wz0,Wz0); Wz.u = {'x1(2)','x2(2)','x3(2)'}; Wz.y = 'z5';

%% Generate the interconnected system

% Define the closed-loop system with tunable parameters
T0 = connect(G1,G2,G3,Ge1,Ge2,Ge3,...
    Ke1,Kv1,Ke2,Kv21,Kv20,Ke3,Kv32,Kv31,... % tunable gains
    We1,We2,We3,...
    Wu,...
    Wz,...
    sum_e1,sum_v10,sum_e2,sum_v21,sum_v20,sum_e3,sum_v32,sum_y31,...
    sum_u1,sum_u2,sum_u3,...
    {'v0'},{'z1','z2','z3','z4','z5'})

%% Structured H infinity synthesis

% hinfstruct config
% rng('default')
rng('shuffle')
opt = hinfstructOptions('Display','final','RandomStart',10);

% Obtaint the closed-loop system with tunned parameters
[T,gamma,info] = hinfstruct(T0,opt);

% Extract
Ke1 = getBlockValue(T,'Ke1');
Kv1 = getBlockValue(T,'Kv1');
K1 = [Ke1 Kv1]

Ke2 = getBlockValue(T,'Ke2');
Kv21 = getBlockValue(T,'Kv21');
Kv20 = getBlockValue(T,'Kv20');
K2 = [Ke2 Kv21 Kv20]

Ke3 = getBlockValue(T,'Ke3');
Kv32 = getBlockValue(T,'Kv32');
Kv31 = getBlockValue(T,'Kv31');
K3 = [Ke3 Kv32 Kv31]
%% Data analysis
% Extract data
w = logspace(-4,3,100);

% v_0->e_i
S11 = T(1,1)/We_e1;
S12 = T(3,1)/We_e2;
S13 = T(6,1)/We_e3;
[sv1,~] = sigma(1/We_e1,w);
[sv2,~] = sigma(1/We_e2,w);
[sv3,~] = sigma(1/We_e3,w);
[sv4,~] = sigma(S11,w);
[sv5,~] = sigma(S12,w);
[sv6,~] = sigma(S13,w);

v0_to_e = [w; 20*log10(sv1); 20*log10(sv4); 20*log10(sv2); 20*log10(sv5);20*log10(sv3); 20*log10(sv6)];

% v_0->v_i-v_{i-1}
Sv10 = T(2,1)/We_v1;
Sv21 = T(4,1)/We_v1;
Sv32 = T(7,1)/We_v1;
% v_0->v_i-v_{i-2}
Sv20 = T(5,1)/We_v2;
Sv31 = T(8,1)/We_v2;

[sv1,~] = sigma(1/We_v1,w);
[sv2,~] = sigma(Sv10,w);
[sv3,~] = sigma(Sv21,w);
[sv4,~] = sigma(Sv32,w);
[sv5,~] = sigma(Sv20,w);
[sv6,~] = sigma(Sv31,w);

v0_to_delta_v = [w;20*log10(sv1);20*log10(sv2);20*log10(sv3);20*log10(sv4);20*log10(sv5);20*log10(sv6)];

% v_0->u_i
Ku1 = T(9,1)/Wu0;
Ku2 = T(10,1)/Wu0;
Ku3 = T(11,1)/Wu0;
[sv1,~] = sigma(1/Wu0,w);
[sv2,~] = sigma(Ku1,w);
[sv3,~] = sigma(Ku2,w);
[sv4,~] = sigma(Ku3,w);
v0_to_u = [w; 20*log10(sv1) ;20*log10(sv2) ;20*log10(sv3); 20*log10(sv4)];

% String stability check
ST0_1 = T(12,1)/Wz0;
ST0_2 = T(13,1)/Wz0;
ST0_3 = T(14,1)/Wz0;

ST1_2 = ST0_2*inv(ST0_1);
ST2_3 = ST0_3*inv(ST0_2);

gpeak_1 = getPeakGain(ST0_1,1e-2,[1e-4 1e3]);
gpeak_2 = getPeakGain(ST1_2,1e-2,[1e-4 1e3]);
gpeak_3 = getPeakGain(ST2_3,1e-2,[1e-4 1e3]);
gpeak_sys = max([gpeak_1 gpeak_2 gpeak_3])

[sv1,~] = sigma(1/Wz0,w);
[sv2,~] = sigma(ST0_1,w);
[sv3,~] = sigma(ST1_2,w);
[sv4,~] = sigma(ST2_3,w);
vi_1_to_vi = [w; 20*log10(sv1) ;20*log10(sv2) ;20*log10(sv3); 20*log10(sv4)];

%% Simulation parameters
% Sampling time
Ts = 0.02;

% Simulation mode
% 1: standard velocity profile, -1: velocity with sin disturbance
sim_mode = 1;
% sinusoidal frequency
dis_freq = 4.5;

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
v_ref0 = 5;
traction_accel = 0.5;
bracking_accel = -0.5;

dt = [40 15 30 10 20 5 10 15 0]'; % Define velocity periods
wp = tril(ones(length(dt)))*dt;
accel = [traction_accel 0 bracking_accel 0 bracking_accel 0 traction_accel 0]; %

%% Time response
sim("test_05_cacc_sim.slx")
% Time
sim_time = tout;

% Spacing error
e_1 = spacing_error.Data(:,1);
e_2 = spacing_error.Data(:,2);
e_3 = spacing_error.Data(:,3);
e_plot = [sim_time e_1 e_2 e_3];
e_plot_ds = downsample(e_plot,20);
e_plot_ds = e_plot_ds';

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
v_plot = [sim_time v_0 v_1 v_2 v_3];
v_plot_ds = downsample(v_plot,20);
v_plot_ds = v_plot_ds';

% Accel
a_1 = acceleration.Data(:,1);
a_2 = acceleration.Data(:,2);
a_3 = acceleration.Data(:,3);
a_des = acceleration.Data(:,4);
a_plot = [sim_time a_1 a_2 a_3];
a_plot_ds = downsample(a_plot,20);
a_plot_ds = a_plot_ds';

% Velocity error
e_v1 = v_1-v_0;
e_v2 = v_2-v_1;
e_v3 = v_3-v_2;

%% Plot - frequency-domain
figure
% From v0 to ei
subplot(231); bodemag(w,1/We_e1,S11);title('From v0 to e1');grid on
subplot(232); bodemag(w,1/We_e2,S12);title('From v0 to e2');grid on
subplot(233); bodemag(w,1/We_e3,S13);title('From v0 to e3');grid on
% legend('1/We_e','Se1','Se2','Se3');title('From v0 to ei');grid on

% String stability check
subplot(234)
bodemag(w,1/Wz0,ST0_1,ST1_2,ST2_3)
legend('1/Wz','ST1','ST2','ST3');title('String stability check'),grid on

% From v0 to vi-v(i-1)
subplot(235)
bodemag(w,1/We_v1,Sv10,Sv21,Sv32)
legend('1/We_v1','Sv10','Sv21','Sv32');title('From v0 to vi-v(i-1)');grid on

% From v0 to vi-v(i-2)
subplot(236)
bodemag(w,1/We_v2,Sv20,Sv31)
legend('1/We_v2','Sv20','Sv31');title('From v0 to vi-v(i-2)');grid on

% bodemag(w,1/Wu0,Ku1,Ku2,Ku3)
% legend('1/Wu0','Ku1','Ku2','Ku3');title('From v0 to ui');grid on
figure
bodemag(w,1/Wu0,Ku1,Ku2,Ku3);title('From v0 to u1');grid on

%% Plot - Time
xlim_max = 120;
xlim_min = 35;
lw = 1.0;

figure
subplot(221)
plot(sim_time, v_0, ...
    sim_time, v_1, ...
    sim_time, v_2, ...
    sim_time, v_3, ...
    LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-22 26])
legend("car_0", "car_1", "car_2", "car_3");
ylabel('v_i(m/s)')
grid on

subplot(222)
plot(sim_time, e_1, ...
    sim_time, e_2, ...
    sim_time, e_3, ...
    LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-1 1])
legend("car_1", "car_2", "car_3");
xlabel('t(s)')
ylabel('e_i(m)')
grid on

subplot(223)
plot(sim_time, d_1, ...
    sim_time, d_2, ...
    sim_time, d_3, ...
    LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-15 15])
legend("car_1", "car_2", "car_3");
xlabel('t[s]')
ylabel('d_i[m]')
grid on

subplot(224)
plot(sim_time, a_1, ...
    sim_time, a_2, ...
    sim_time, a_3, ...
    LineWidth=lw); xlim([xlim_min xlim_max]); %ylim([-1.5 1.5])
legend("a1", "a2", "a3");
xlabel('t(s)')
ylabel('a_i(m/s^2)')
grid on

%% Export data
export_data=false;

if export_data
    fig_loc = '/home/phatdo/01_master_mars/01_Master_Thesis/02_Report/master_report_cacc/dat/';
    
    fig_name = 'chap5_homo_freq_v0_to_e_2PF.txt';
    save(append(fig_loc, fig_name)); % Create a new .txt file
    fileID = fopen(append(fig_loc, fig_name),"w");
    fprintf(fileID,' %s %s %s %s %s %s %s\n','w','1/We1','e1','1/We2','e2','1/We3','e3');
    fprintf(fileID,' %e %e %e %e %e %e %e\r\n',v0_to_e);
    fclose(fileID);
    
    % Vel
    fig_name = 'chap5_homo_freq_v0_to_delta_v_2PF.txt'; % Create a new .txt file
    save(append(fig_loc, fig_name));
    fileID = fopen(append(fig_loc, fig_name),"w");
    fprintf(fileID,' %s %s %s %s %s %s %s\n','w','1/We_v1','v10','v21','v32','v20','v31');
    fprintf(fileID,' %e %e %e %e %e %e %e\r\n',v0_to_delta_v);
    fclose(fileID);
    
    % Control input
    fig_name = 'chap5_homo_freq_v0_to_u_2PF.txt';
    save(append(fig_loc, fig_name)); % Create a new .txt file
    fileID = fopen(append(fig_loc, fig_name),"w");
    fprintf(fileID,' %s %s %s %s %s\n','w','1/Wu','u1','u2','u3');
    fprintf(fileID,' %e %e %e %e %e\r\n',v0_to_u);
    fclose(fileID);
    
    % String stability
    fig_name = 'chap5_homo_freq_string_2PF.txt';
    save(append(fig_loc, fig_name)); % Create a new .txt file
    fileID = fopen(append(fig_loc, fig_name),"w");
    fprintf(fileID,' %s %s %s %s %s\n','w','1/Wz','gam1','gam2','gam3');
    fprintf(fileID,' %e %e %e %e %e\r\n',vi_1_to_vi);
    fclose(fileID);

     % Get accel
    fig_name = 'chap5_homo_time_acc_2pf.txt';
    save(append(fig_loc, fig_name)); % Create a new .txt file
    fileID = fopen(append(fig_loc, fig_name),"w");
    fprintf(fileID,' %s %s %s %s \n','t','a1','a2','a3');
    fprintf(fileID,' %e %e %e %e \r\n',a_plot_ds);
    fclose(fileID);
    
    % Get vel
    fig_name = 'chap5_homo_time_vel_2pf.txt';
    save(append(fig_loc, fig_name)); % Create a new .txt file
    fileID = fopen(append(fig_loc, fig_name),"w");
    fprintf(fileID,' %s %s %s %s %s\n','t','v0','v1','v2','v3');
    fprintf(fileID,' %e %e %e %e %e\r\n',v_plot_ds);
    fclose(fileID);
    
    % Get error
    fig_name = 'chap5_homo_time_err_2pf.txt';
    save(append(fig_loc, fig_name)); % Create a new .txt file
    fileID = fopen(append(fig_loc, fig_name),"w");
    fprintf(fileID,' %s %s %s %s\n','t','e1','e2','e3');
    fprintf(fileID,' %e %e %e %e\r\n',e_plot_ds);
    fclose(fileID);
    disp("-----------> Data exported...")
else
    disp("-----------> No data exported...")
end