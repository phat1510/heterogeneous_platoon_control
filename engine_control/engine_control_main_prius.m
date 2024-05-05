% File name: engine_control_main_prius.m
% Dependencies: 
%   engine_control_main_sim.slx
%   engine_controller_synthesis.m
% Author: Phat Do
% Apr 2024
% Description: H-infinity controller synthesis for a car engine including
% accel/braking model

%% Reset the workspace
clc
clear 
close all;

%% Load all models
load('models/G_prius.mat');

[K_a, K_b] = get_controller(G_prius_a_2nd, G_prius_b_2nd);


%% Simulation

% Simulation variables
G_a = G_prius_a; G_b = G_prius_b;
G_a_2nd = G_prius_a_2nd; G_b_2nd = G_prius_b_2nd;

sim("engine_control_main_sim.slx");
sim_time = tout;

accel_input = traction.Data(:,1);
accel_ref   = traction.Data(:,2);
accel_ctrl  = traction.Data(:,3);
accel_idtf  = traction.Data(:,4);
accel_pade  = traction.Data(:,5);

decel_input = braking.Data(:,1);
decel_ref   = braking.Data(:,2);
decel_ctrl  = braking.Data(:,3);
decel_idtf  = braking.Data(:,4);
decel_pade  = braking.Data(:,5);

%% Visualization
figure
lw = 1;
subplot(211)
plot(sim_time, accel_ref,...
    sim_time,accel_idtf,...
    sim_time,accel_ctrl,...
    sim_time,accel_input,...
    LineWidth=lw); 
legend("reference", ...
    "output - open-loop - identified model", ...
    "closed-loop",...
    "control input");

xlabel('t[s]'), xlim([0 5])
ylabel('a_i[m/s^2]')
grid on

subplot(212)
plot(sim_time, decel_ref,...
    sim_time,decel_idtf,...
    sim_time,decel_ctrl,...
    sim_time,decel_input,...
    LineWidth=lw); 
legend("reference", ...
    "output - open-loop - identified model", ...
    "closed-loop",...
    "control input");

xlabel('t[s]'), xlim([0 5])
ylabel('a_i[m/s^2]')
grid on


function [K_a,K_b] = get_controller(G_accel, G_decel)
    %% Controller design
    % For tracking error (S)
    Ms=2;wb=1.1;epsi=0.001;
    We=tf([1/Ms wb],[1 wb*epsi]);
    
    % For control input (KS)
    Mu=1.5;wbc=75;epsi1=0.01;
    % Wu=tf([1 wbc/Mu],[epsi1 wbc]);
    Wu = tf(1/Mu);
    
    % For input disturbance
    Wd=ss(0.15);
    
    % Noise filter
    % Wn=(3*s+150)/(s+2000); 
    Wn=tf(1); % Dont filter
    
    % Accel
    [K_a, CL_a, gamma_a] = engine_controller_synthesis(G_accel, We, Wu, Wd, Wn);
    frequency_response(G_accel, CL_a, We, Wu);
    
    % --------------------------------------------------------------
    % For tracking error (S)
    Ms=2;wb=1.1;epsi=0.001;
    We=tf([1/Ms wb],[1 wb*epsi]);
    
    % For control input (KS)
    Mu=1.5;wbc=75;epsi1=0.01;
    % Wu=tf([1 wbc/Mu],[epsi1 wbc]);
    Wu=tf(1/Mu);
    
    % For input disturbance
    Wd=ss(0.15);
    
    % Noise filter
    % Wn=(3*s+150)/(s+2000); 
    Wn=tf(1); % Don't filter
    
    % Braking
    [K_b, CL_b, gamma_b] = engine_controller_synthesis(G_decel, We, Wu, Wd, Wn);
    frequency_response(G_decel, CL_b, We, Wu);

    % Export the controllers
    K_prius_a = K_a;
    K_prius_b = K_b;
    save('controllers/K_prius.mat','K_prius_a','K_prius_b');
end

function frequency_response(G_engine, CL, We, Wu)
    w = logspace(-4,3,100);
    S = CL(1,1)/We;
    KS = CL(2,1)/Wu;
    T = 1-S;
    SG = S*G_engine;
    
    figure
    subplot(221)
    bodemag(w,1/We,S); grid on,
    legend('1/We','S');
    
    subplot(222)
    bodemag(w,T); grid on,
    legend('T');
    
    subplot(223)
    bodemag(w,SG); grid on,
    legend('SG');
    
    subplot(224)
    bodemag(w,1/Wu,KS); grid on,
    legend('1/Wu','KS');
end

