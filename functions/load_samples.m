function [accel_u, accel_y, time_frame] = load_samples(file_name,s1,s2)
A = importdata(file_name);
month = A(:,1);
day = A(:,2);
year = A(:,3);
hour = A(:,4);
min =  A(:,5);

sec = A(:,6);
msec = A(:,7);
cur_time = A(:,8);
cur_control_mode2 = A(:,9); % modes and transitions as in 661 conditions
v2i_engaged = A(:,10); 

cur_control_mode = A(:,11); % cc = 0, acc = 1, the rest transients to be seen from the long_control.hpp 
my_pip = A(:,12);
ego_vehicle_id = A(:,13);
long_speed = A(:,14);
fuel_rate = A(:,15);

long_accel = A(:,16);
gps_data_latitude = A(:,17);
gps_data_longitude = A(:,18);
gps_data_altitude = A(:,19);
is_prec_valid = A(:,20);

prec_veh_rel_distance = A(:,21);
prec_veh_rel_speed = A(:,22);
prec_veh_rel_pos = A(:,23);
throttle_lev_com = A(:,24);
brake_lev_com = A(:,25);

dec_com = A(:,26); %difference with the negative values of A(:,16) ? Filtered? 
set_speed = A(:,27);
des_accel = A(:,28);
des_dist_gap = A(:,29);
ACC_time_gap = A(:,30);

ACC_gap_error = A(:,31);
ACC_fb_out = A(:,32);
ACC_ref_speed = A(:,33);
v2v_fault_timer = A(:,34);
cooperative_ACC_time_gap = A(:,35);

CACC_gap_error = A(:,36);
cooperative_ACC_fb_out = A(:,37);
cooperative_ACC_ref_speed = A(:,38);
preceding_ref_speed = A(:,39);
leader_ref_speed = A(:,40);

is_LPF_active = A(:,41);
CACC_ff_out = A(:,42);
v2v0_pip = A(:,43);
v2v0_id = A(:,44);
v2v0_ref_speed = A(:,45);

v2v0_ref_accel = A(:,46);
v2v0_long_speed = A(:,47);
v2v0_accel = A(:,48);
v2v0_coordinate = A(:,49);
v2v0_control_mode = A(:,50);

v2v1_pip = A(:,51);
v2v1_ID = A(:,52);
v2v1_ref_speed = A(:,53);
v2v1_ref_accel = A(:,54);
v2v1_long_speed = A(:,55);

v2v1_accel = A(:,56);
v2v1_coordinate = A(:,57);
v2v1_control_mode = A(:,58);
v2v2_pip = A(:,59);
v2v2_id = A(:,60);

v2v2_ref_speed = A(:,61);
v2v2_ref_accel = A(:,62);
v2v2_long_speed = A(:,63);
v2v2_accel = A(:,64);
v2v2_coordinate = A(:,65);

v2v2_control_mode = A(:,66);

brake_switch = A(:,77);
%brake_position = A(:,78);
%brake_pressure = A(:,79);
v2v0_brake_switch = A(:,78);
v2v1_brake_switch = A(:,79);
v2v2_brake_switch = A(:,80);
trigger_EB = A(:,81);
is_cacc_perf_flag = A(:,82);

brake_position = A(:,83);
v2v0_brake_position = A(:,84);
v2v1_brake_position = A(:,85);
v2v2_brake_position = A(:,86);

brake_pressure = A(:,87);
v2v0_brake_pressure = A(:,88);
v2v1_brake_pressure = A(:,89);
v2v2_brake_pressure = A(:,90);

cc_active = A(:,91);
v2v0_cc_active = A(:,92);
v2v1_cc_active = A(:,93);
v2v2_cc_active = A(:,94);


%figure,
%plot(cur_time,2.2369*long_speed);% in meters per seconds
%hold on
%plot(cur_time,2.2369*set_speed);

%figure,
%plot(cur_time,long_accel);% in meters per seconds^2
%hold on
%plot(cur_time,des_accel)

%figure,
%plot(cur_time,prec_veh_rel_distance)
%%%
%figure,
%plot(cur_time,prec_veh_rel_speed)% vx_rel = vx,o - vx,e .. i.e vx_rel <0 ; i go faster than the other car so i brake under acc to regulate distance. 
%%%
%figure,
%plot(cur_time,is_prec_valid)
%figure,
%plot(cur_time,prec_veh_rel_speed)
%close all
% figure,
%plot(cur_time,brake_switch)
%hold on
% plot(cur_time,v2v0_brake_switch)
% legend('v2v0 brake switch')
%plot(cur_time,v2v1_brake_switch)
% ylim([-0.1 1.1])
%plot(cur_time,v2v2_brake_switch)
%legend('brake switch from input struct','v2v0 brake switch','v2v1 brake switch', 'v2v2 brake switch')

sample_t1 = s1;
sample_t2 = s2;

accel_u = des_accel(sample_t1:sample_t2,1);
accel_y = v2v0_accel(sample_t1:sample_t2,1);
time_frame = cur_time(sample_t1:sample_t2,1);

end