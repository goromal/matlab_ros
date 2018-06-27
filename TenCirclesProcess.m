clear all
close all
% Choose Bag
% data = './Bags/ten_circles_first.bag';
% data = './Bags/ten_circles_second.bag';
data = './Bags/ten_circles_third.bag';

% Get data
data = processAllTopics(data);

%% Data processing
close all

% Odom Stuff
ins_pos = data.ins.pose.position;
ins_N = ins_pos(1,:);
ins_E = ins_pos(2,:);
ins_D = ins_pos(3,:);
ins_quat = data.ins.pose.orientation;

% Zero the center
ins_nN = ins_N - mean(ins_N);
ins_nE = ins_E - mean(ins_E);
ins_nD = ins_D - mean(ins_D);

% GPS stuff
ins_gps = data.gps;
ins_lat = ins_gps.lat;
ins_lon = ins_gps.lon;

% 
% % Plot INS Position
% figure(1)
% plot3(ins_nE, ins_nN, -ins_nD)
% axis equal
% hold on

% Plot Lat/Lon
figure(2)
plot(ins_lat, ins_lon)




