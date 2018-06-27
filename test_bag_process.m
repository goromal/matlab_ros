
first_data = './Bags/blind_flight_2017-08-01-FIRST.bag';
% second_data = blind_flight_2017-08-01-SECOND.bag;
data = processAllTopics(first_data)

% proper_gains = 'proper_gains_2017-03-04-18-29-58.bag';
% test_flight = 'gps_new_test2_2016-02-11-16-30-18.bag';
% airspeed1 = 'airspeedtest1_2016-02-11-16-33-01.bag';
% move_ftdi = 'airspeedtestmovewire_2016-02-11-16-35-50.bag';
% speed_long = 'airspeedtest_long_2016-02-11-16-54-57.bag';
% dw ='david_walk_2_2016-02-11-16-39-49.bag';
% dane1 = 'dane_outside_2016-02-11-16-57-36.bag';
% dane3 = 'dane_outside_3_2016-02-11-17-01-49.bag';
% airspeed_init = 'airspeed_init_2016-02-11-16-29-19.bag';
% air = 'air_test_2016-02-11-16-31-27.bag';
% rc1 = 'rc_flight_1_2016-02-11-16-45-38.bag';
% rc2 = 'rc_flight_2_2016-02-11-16-35-24.bag';
% anton = 'anthony_walk_2016-02-11-16-36-35.bag';
% new_air = 'airspeed_test_3_2016-02-11-16-55-29.bag';
% crash = 'test_autopilot_2016-02-11-16-53-38.bag';
% gps = 'test_fly_may_1_3_2016-02-11-16-38-27.bag';
% north = 'climb_north_7_2016-02-11-16-55-08.bag';
% north2 = 'climb_north_8_2016-02-11-17-01-40.bag';
% auto = 'auto_1_2016-02-11-16-30-48.bag';
% loiter = 'prefly_2016-02-11-16-31-53.bag';

% data = processTopics({'/attitude', '/state', '/baro','/gps/data','/airspeed', '/command','/rc_raw','/current_path','/controller_commands', '/controller_inners'}, loiter);
    
%data = processTopics({'/rc_raw'}, crash);
%data1 = processTopics({'/state','/airspeed'}, airspeed1);
%data2 = processTopics({'/state','/airspeed'}, move_ftdi);
%data3 = processTopics({'/state','/airspeed'}, speed_long);

% for m = 1:length(data.state.position(2,:))
%     if abs(data.state.position(1,m)) > 300
%         data.state.position(1,m) = 0; 
%     end
%     if abs(data.state.position(2,m)) > 300 
%         data.state.position(2,m) = 0; 
%     end
% end

% 
%  figure(1)
%  plot(data.state.position(3,:))
%  title('state position down')
%  figure(2)
%  plot(data.controller_commands.h_c)
%  title('h_c')
%  figure(3)
%  plot(data.controller_inners.alt_zone)
%  title('alt_zone')
 
%  figure(4)
%  plot(data.current_path.Va_d)
%  title('Va_d')
% 
% figure(5)
% plot(data.state.position(2,:))
% title('state position east')
% figure(6)
% plot(data.state.position(2,:),data.state.position(1,:))
% title('state position north')
% 
% figure(4)
% plot(data.gps.data.latitude)
% title('latitude')
% % % 
% figure(5)
% plot(data.gps.data.longitude)
% title('longitude')

%  figure(6)
% 
%  plot(data.state.chi)
%  title('chi')
%  
%  figure(7)
%  plot(data.state.chi_deg)
%  title('chi_deg')
 
%  
% 
% figure(9)
% plot(data.state.phi)
% title('State Phi')
% 
% figure(10)
% plot(data.state.theta)
% title('State Theta')

% figure(6)
% plot(data.state.Vg)
% title('State Vg')
% hold on
% plot(data.command.F)
% 
% figure(7)
% plot(data.command.x)
% title('command x')
% 
% figure(8)
% plot(data.command.y)
% title('command y')

% figure(9)
% plot(data.rc_raw.rc(1,:))
% title('channel 0')
% 
% figure(10)
% plot(data.rc_raw.rc(2,:))
% title('channel 1')
% 
% figure(11)
% plot(data.rc_raw.rc(3,:))
% title('channel 2')
% 
% figure(13)
% plot(data.rc_raw.rc(4,:))
% title('channel 3')
% 
% time = 1:2:2*length(data.rc_raw.rc(5,:));
% figure(14)
% plot((data.rc_raw.rc(5,:)-1675)./230)
% title('channel 4 (auto)')
% 
% plot(data.gps.data.NumSat)
% title('NumSat')
% figure(15)
% plot(data.rc_raw.rc(6,:))
% title('channel 5 (arm)')

% 
% figure(11)
% plot(data.command.F)
% title('command.F')

% figure(11)
% plot(data.command.MODE_XPOS_YPOS_YAW_ALTITUDE)
% title('command.MODE XPOS YPOS YAW ALTITUDE')
% 
% figure(12)
% plot(data.command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE)
% title('command.MODE XVEL YVEL YAWRATE ALTITUDE')
% 
% figure(13)
% plot(data.airspeed.velocity) 
% title('Airspeed')
% 
% figure(12)
% plot(data.state.velocity)
% title('State Velocity')

% figure(15)
% plot(data.airspeed.differential_pressure)
% title('Diff Press')
% 
% figure(16)
% plot(data.gps.data.NumSat)
% title('NumSat')
% % 
% figure(17)
% plot(-data.baro.altitude(33000:end))
% title('Altitude')
