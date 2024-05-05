clear
clc
close all

% Note: Set current folder to /engine_modeling before start


%% Leaf's engine
get_leaf();
load('data/G_leaf.mat');
load('data/G_leaf_samples.mat');

% Test in Simulink
% Simulink variables:
% Sampling time
Ts = 0.02;
accel_test = 1;
brake_test = -2;

% Model with delay
G_a = G_leaf_a;
G_b = G_leaf_b;
% 1st order approximated model
G_a_1st = G_leaf_a_1st;
G_b_1st = G_leaf_b_1st;

% 2nd order approximated model
G_a_2nd = G_leaf_a_2nd;
G_b_2nd = G_leaf_b_2nd;

sim('open_loop_sim.slx')

sample_t1 = 428; sample_t2 = 600;
sample_no = sample_t2-sample_t1;
sample_t1 = 450; % correct the time frame

sim_time = tout(sample_t1:(sample_t1+sample_no),:);

sim_a = sim_a.Data(sample_t1:(sample_t1+sample_no),:);
sim_b = sim_b.Data(sample_t1:(sample_t1+sample_no),:);

% Spacing error
accel_traction_ref  = sim_a(:,1);
accel_traction      = sim_a(:,2);
accel_traction_st   = sim_a(:,3);
accel_traction_nd   = sim_a(:,4);

accel_braking_ref   = sim_b(:,1);
accel_braking       = sim_b(:,2);
accel_braking_st    = sim_b(:,3);
accel_braking_nd    = sim_b(:,4);

% Visualization
figure('Position',[500 500 1200 900])
lw = 1.2;
subplot(321)
plot(traction_time, traction_u, ...
    traction_time, traction_y, ...
    sim_time, accel_traction,...
    sim_time, accel_traction_st,...
    sim_time, accel_traction_nd,...
    LineWidth=lw); 
legend("a desired", ...
    "a measured ", ...
    "first-order with delay", ...
    "first-order model",...
    "second-order model");
xlabel('t[s]'); ylabel('a_i[m/s^2]');
xlim([traction_time(1) traction_time(length(traction_time))]);
legend('Location','best')
grid on

subplot(322)
plot(braking_time, braking_u, ...
    braking_time, braking_y, ...
    sim_time,accel_braking,...
    sim_time, accel_braking_st,...
    sim_time, accel_braking_nd,...
    LineWidth=lw); 
legend("a desired", ...
    "a measured ", ...
    "first-order with delay", ...
    "first-order model",...
    "second-order model");
xlabel('t[s]'); ylabel('a_i[m/s^2]');
xlim([braking_time(1) braking_time(length(braking_time))]);
grid on

%% Prius
clear
get_prius();
load('data/G_prius.mat');
load('data/G_prius_samples.mat');

% Test in Simulink
% Simulink variables
% Sampling time
Ts = 0.02;
accel_test = 1;
brake_test = -1.5;

% Model with delay
G_a = G_prius_a;
G_b = G_prius_b;
% 1st order approximated model
G_a_1st = G_prius_a_1st;
G_b_1st = G_prius_b_1st;

% 2nd order approximated model
G_a_2nd = G_prius_a_2nd;
G_b_2nd = G_prius_b_2nd;

% Run sim
sim('open_loop_sim.slx')

sample_a_no = sample_a_t2-sample_a_t1;
sample_b_no = sample_b_t2-sample_b_t1;

sim_time = tout;

sim_time_a = sim_time(sample_a_t1:(sample_a_t1+sample_a_no),:);
sim_a = sim_a.Data(sample_a_t1:(sample_a_t1+sample_a_no),:);


sim_time_b = sim_time(sample_b_t1:(sample_b_t1+sample_b_no),:);
sim_b = sim_b.Data(sample_b_t1:(sample_b_t1+sample_b_no),:);

% Spacing error
accel_traction_ref = sim_a(:,1);
accel_traction = sim_a(:,2);
accel_traction_st = sim_a(:,3);
accel_traction_nd = sim_a(:,4);

accel_braking_ref = sim_b(:,1);
accel_braking = sim_b(:,2);
accel_braking_st = sim_b(:,3);
accel_braking_nd = sim_b(:,4);

% Visualization
lw = 1.2;
subplot(323)
plot(traction_time, traction_u, ...
    traction_time, traction_y, ...
    sim_time_a, accel_traction,...
    sim_time_a, accel_traction_st,...
    sim_time_a, accel_traction_nd,...
    LineWidth=lw); 
legend("a desired", ...
    "a measured ", ...
    "first-order with delay", ...
    "first-order model",...
    "second-order model");
xlabel('t[s]'); ylabel('a_i[m/s^2]');
xlim([traction_time(1) traction_time(length(traction_time))]);
legend('Location','best')
grid on

subplot(324)
plot(braking_time, braking_u, ...
    braking_time, braking_y, ...
    sim_time_b,accel_braking,...
    sim_time_b, accel_braking_st,...
    sim_time_b, accel_braking_nd,...
    LineWidth=lw); 
legend("a desired", ...
    "a measured ", ...
    "first-order with delay", ...
    "first-order model",...
    "second-order model");
xlabel('t[s]'); ylabel('a_i[m/s^2]');
xlim([braking_time(1) braking_time(length(braking_time))]);
grid on

%% Camry
clear
get_camry();
load('data/G_camry.mat');
load('data/G_camry_samples.mat');

% Test in Simulink
% Simulink variables
% Sampling time
Ts = 0.02;
accel_test = 1;
brake_test = -2;

% Model with delay
G_a = G_camry_a;
G_b = G_camry_b;
% 1st order approximated model
G_a_1st = G_camry_a_1st;
G_b_1st = G_camry_b_1st;

% 2nd order approximated model
G_a_2nd = G_camry_a_2nd;
G_b_2nd = G_camry_b_2nd;

% Run sim
sim('open_loop_sim.slx')

sample_a_no = sample_a_t2-sample_a_t1;
sample_b_no = sample_b_t2-sample_b_t1;

sim_time = tout;

sim_time_a = sim_time(sample_a_t1:(sample_a_t1+sample_a_no),:);
sim_a = sim_a.Data(sample_a_t1:(sample_a_t1+sample_a_no),:);


sim_time_b = sim_time(sample_b_t1:(sample_b_t1+sample_b_no),:);
sim_b = sim_b.Data(sample_b_t1:(sample_b_t1+sample_b_no),:);

% Spacing error
accel_traction_ref = sim_a(:,1);
accel_traction = sim_a(:,2);
accel_traction_st = sim_a(:,3);
accel_traction_nd = sim_a(:,4);

accel_braking_ref = sim_b(:,1);
accel_braking = sim_b(:,2);
accel_braking_st = sim_b(:,3);
accel_braking_nd = sim_b(:,4);

% Visualization
lw = 1.2;
subplot(325)
plot(traction_time, traction_u, ...
    traction_time, traction_y, ...
    sim_time_a, accel_traction,...
    sim_time_a, accel_traction_st,...
    sim_time_a, accel_traction_nd,...
    LineWidth=lw); 
legend("a desired", ...
    "a measured ", ...
    "first-order with delay", ...
    "first-order model",...
    "second-order model");
xlabel('t[s]'); ylabel('a_i[m/s^2]');
xlim([traction_time(1) traction_time(length(traction_time))]);
legend('Location','best')
grid on

subplot(326)
plot(braking_time, braking_u, ...
    braking_time, braking_y, ...
    sim_time_b,accel_braking,...
    sim_time_b, accel_braking_st,...
    sim_time_b, accel_braking_nd,...
    LineWidth=lw); 
legend("a desired", ...
    "a measured ", ...
    "first-order with delay", ...
    "first-order model",...
    "second-order model");
xlabel('t[s]'); ylabel('a_i[m/s^2]');
xlim([braking_time(1) braking_time(length(braking_time))]);
grid on

%% Export data

% data_array_ga = [sim_time traction_time traction_u traction_y accel_traction accel_traction_st accel_traction_nd]';
% data_array_gb = [sim_time braking_time braking_u braking_y accel_braking accel_braking_st accel_braking_nd]';

% fig_loc = '/home/phatdo/01_master_mars/01_Master_Thesis/02_Report/master_report_cacc/dat/';
% fig_name = 'leaf_engine_traction_data.txt';
% save(append(fig_loc, fig_name)); % Create a new .txt file
% fileID = fopen(append(fig_loc, fig_name),"w");
% fprintf(fileID,' %s %s %s %s %s %s %s\n','ts','tm','u','y','g1','g2','g3');
% fprintf(fileID,' %e %e %e %e %e %e %e\r\n',data_array_ga);
% fclose(fileID);
% 
% fig_name = 'leaf_engine_braking_data.txt';
% save(append(fig_loc, fig_name)); % Create a new .txt file
% fileID = fopen(append(fig_loc, fig_name),"w");
% fprintf(fileID,' %s %s %s %s %s %s %s\n','ts','tm','u','y','g1','g2','g3');
% fprintf(fileID,' %e %e %e %e %e %e %e\r\n',data_array_gb);
% fclose(fileID);


function get_leaf()
    % Get traction model
    traction_file = 'data/accel_leaf.dat';
    
    % Data extraction
    sample_t1 = 428; sample_t2 = 600;
    [traction_u, traction_y, traction_time] = load_samples(traction_file,sample_t1,sample_t2);
    
    % System identification
    Ts = 0.02;
    G_leaf_a = model_estimation(traction_u,traction_y,Ts);
    
    % Get braking model
    braking_file = 'data/brake_leaf_1.dat';
    % braking_file = 'data/brake_leaf_2.dat';
    
    % Data extraction
    [braking_u, braking_y, braking_time] = load_samples(braking_file,sample_t1,sample_t2);
    
    % System identification
    G_leaf_b = model_estimation(braking_u,braking_y,Ts);
    
    % Model approximation
    % First order
    s = tf('s');
    tau = 0.3; bias = 0.87; G_leaf_a_1st = bias/(tau*s +1);
    tau = 0.5; bias = 1.4; G_leaf_b_1st = bias/(tau*s +1);
    
    % Second order (pade) 
    G_leaf_a_2nd = pade(G_leaf_a,1);
    G_leaf_b_2nd = pade(G_leaf_b,1);
    
    % Save model to disk
    save('../data/G_leaf.mat','G_leaf_a','G_leaf_b','G_leaf_a_1st','G_leaf_a_2nd','G_leaf_b_1st','G_leaf_b_2nd');
%     save('../data/G_leaf_samples.mat', ...
%         'braking_time','braking_u','braking_y', ...
%         'traction_time','traction_u','traction_y', ...
%         'sim_time', ...
%         'accel_traction_ref','accel_traction','accel_traction_st','accel_traction_nd', ...
%         'accel_braking_ref','accel_braking','accel_braking_st','accel_braking_nd');
    save('../data/G_leaf_samples.mat', ...
    'braking_time','braking_u','braking_y', ...
    'traction_time','traction_u','traction_y');
end

function get_prius()  
    % Get traction model
    % traction_file = 'prius_accel.dat';
    traction_file = 'data/prius_2_accel.dat';
    
    % Data extraction
    sample_a_t1 = 450; sample_a_t2 = 751;
    [traction_u, traction_y, traction_time] = load_samples(traction_file, ...
                                                          sample_a_t1,sample_a_t2);
    
    % System identification
    Ts = 0.02;
    G_prius_a = model_estimation(traction_u,traction_y,Ts);
    % G_a =exp(-0.52*s)*(0.6/(s+0.6));
    
    % Get braking model
    % braking_file = 'prius_brake_1.dat';
    % braking_file = 'prius_brake_2.dat';
    braking_file = 'data/prius_2_decel.dat';
    
    % Data extraction
    sample_b_t1 = 450; sample_b_t2 = 700;
    [braking_u, braking_y, braking_time] = load_samples(braking_file, ...
                                                         sample_b_t1,sample_b_t2);
    % System identification
    
    G_prius_b = model_estimation(braking_u,braking_y,Ts);
    
    % Get data for visualization
%     sample_b_t1 = 450; sample_b_t2 = 750;
%     [braking_u, braking_y, braking_time] = load_samples(braking_file, ...
%                                                          sample_b_t1,sample_b_t2);
    
    % Model approximation
    % First order
    s = tf('s');
    tau = 0.7; bias = 1.14; G_prius_a_1st = bias/(tau*s +1);
    tau = 0.85; bias = 1.1; G_prius_b_1st = bias/(tau*s +1);
    
    % Second order (pade) 
    G_prius_a_2nd = pade(G_prius_a,1);
    G_prius_b_2nd = pade(G_prius_b,1);

    % Save model to disk
    save('../data/G_prius.mat','G_prius_a','G_prius_b','G_prius_a_1st','G_prius_a_2nd','G_prius_b_1st','G_prius_b_2nd');
    save('../data/G_prius_samples.mat', ...
        'braking_time','braking_u','braking_y', ...
        'traction_time','traction_u','traction_y', ...
        'sample_a_t1', 'sample_a_t2', ...
        'sample_b_t1', 'sample_b_t2');

end

function get_camry()
    % Get traction model
    traction_file = 'data/camry_accel.dat';
%     traction_file = 'data/camry_2_accel.dat';
    
    % Data extraction
    sample_a_t1 = 450; sample_a_t2 = 751;
    [traction_u, traction_y, traction_time] = load_samples(traction_file, ...
                                                          sample_a_t1,sample_a_t2);
    
    % System identification
    Ts = 0.02;
    G_camry_a = model_estimation(traction_u,traction_y,Ts);
    % G_a =exp(-0.52*s)*(0.6/(s+0.6));
    
    % Get braking model
%     braking_file = 'data/camry_brake_1.dat';
    braking_file = 'data/camry_brake_2.dat';
%     braking_file = 'data/camry_2_decel.dat';
    
    % Data extraction
    sample_b_t1 = 450; sample_b_t2 = 750;
    [braking_u, braking_y, braking_time] = load_samples(braking_file, ...
                                                         sample_b_t1,sample_b_t2);
    % System identification
    
    G_camry_b = model_estimation(braking_u,braking_y,Ts);
    
    % Get data for visualization
%     sample_b_t1 = 450; sample_b_t2 = 750;
%     [braking_u, braking_y, braking_time] = load_samples(braking_file, ...
%                                                          sample_b_t1,sample_b_t2);
    
    % Model approximation
    % First order
    s = tf('s');
    tau = 2; bias = 1.2; G_camry_a_1st = bias/(tau*s +1);
    tau = 1.6; bias = 1.25; G_camry_b_1st = bias/(tau*s +1);
    
    % Second order (pade) 
    G_camry_a_2nd = pade(G_camry_a,1);
    G_camry_b_2nd = pade(G_camry_b,1);

    % Save model to disk
    save('../data/G_camry.mat','G_camry_a','G_camry_b','G_camry_a_1st','G_camry_a_2nd','G_camry_b_1st','G_camry_b_2nd');
    save('../data/G_camry_samples.mat', ...
        'braking_time','braking_u','braking_y', ...
        'traction_time','traction_u','traction_y', ...
        'sample_a_t1', 'sample_a_t2', ...
        'sample_b_t1', 'sample_b_t2');
end