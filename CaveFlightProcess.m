clear all
close all
% Choose Bag
data = './Bags/cave1.bag';
% data = './Bags/cave2.bag';
% data = './Bags/cave3.bag';

% Get data
data = processAllTopics(data);

%% Data processing
 
% Extract sensor data
baro = data.baro.altitude;
sonar = data.sonar.range;
acc = data.imu.data.acc;
gyro = data.imu.data.gyro;

% 
% % Plot Baro
% figure(1)
% plot(baro)

% Plot Sonar
figure(2)
plot(sonar)

% Plot acc 
figure(3)
plot(acc(1,:));
hold on
plot(acc(2,:));
plot(acc(3,:));

% Plot gyro
figure(4)
plot(gyro(1,:));
hold on
plot(gyro(2,:));
plot(gyro(3,:));





