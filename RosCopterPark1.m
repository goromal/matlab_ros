clear all
close all
% Choose Bag
data = '~/magicc/rosbags/roscopter_park1/roscopter_spider_2017-08-04-00-00-36.bag';
% data = './Bags/cave2.bag';
% data = './Bags/cave3.bag';

% Get data
data = processAllTopics(data);

%% Data processing
 
% Extract sensor data
baro = data.baro.altitude;
sonar = data.sonar.range;
mag = data.magnetometer.mag;
acc = data.imu.data.acc;
gyro = data.imu.data.gyro;
lat = data.gps.data.latitude;
lon = data.gps.data.longitude;

% 
% % Plot Baro
% figure(1)
% plot(baro(1000:3000))
% 
% % Plot Sonar
% figure(1)
% plot(sonar)
% 
% figure(2)
% plot(sonar,'o')
% 
% % Filter sonar
% D = designfilt('lowpassiir', 'PassbandFrequency', .45, 'StopbandFrequency', .55, 'PassbandRipple', 1, 'StopbandAttenuation', 60);
% filt_sonar = filtfilt(D, double(sonar));
% figure(3)
% plot(filt_sonar,'o');
% 
% % Custon filter sonar
% fil_sonar = [];
% for i=2:length(sonar)-1
%     if (abs(sonar(i)-sonar(i-1))) < 0.2
%         fil_sonar = [fil_sonar, sonar(i)];
%     end
% end
% figure(4)
% plot(fil_sonar,'o');

% % Magnetometer
% figure(5)
% plot(mag(1,:))

% GPS
figure(6)
plot(lat)
plot(lon)

% 
% % Plot acc 
% figure(3)
% plot(acc(1,:));
% hold on
% plot(acc(2,:));
% plot(acc(3,:));
% 
% % Plot gyro
% figure(4)
% plot(gyro(1,:));
% hold on
% plot(gyro(2,:));
% plot(gyro(3,:));





