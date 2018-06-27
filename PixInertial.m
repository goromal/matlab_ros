clear all
close all
% Choose Bag
% data = './Bags/blind_flight_2017-08-01-FIRST.bag';
data = './Bags/blind_flight_2017-08-01-SECOND.bag';

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

% Plot INS Position
figure(1)
plot3(ins_nE, ins_nN, -ins_nD)
axis equal
hold on

% Pixhawk Telem

% load('./PixhawkTelem/blind_first.mat')
load('./PixhawkTelem/blind_second.mat')

pix_N = EKF1(:,9);
pix_E = EKF1(:,10);
pix_D = EKF1(:,11);

% Try to center the points
pix_nN = pix_N - mean(pix_N);
pix_nE = pix_E - mean(pix_E);
pix_nD = pix_D - mean(pix_D);

% Plot Pixhawk Position
% figure(1)
plot3(pix_nE, pix_nN, -pix_nD)
axis equal



