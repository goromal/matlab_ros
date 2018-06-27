clear all
close all
% Choose Bag
% data = '~/magicc/rosbags/planckcam_springville/planck_cam_ins_second.bag';
% data = '~/magicc/rosbags/planckcam_springville/planck_cam_ins_third.bag';
data = '~/magicc/rosbags/planckcam_springville/planck_cam_ins_fourthRC.bag';
% data = '~/magicc/rosbags/planckcam_springville/planck_cam_ins_zero.bag';

% Get data
data = processAllTopics(data);

%% Data processing
 
% INS data
ins_t = data.ins.t;
ins_pos = data.ins.pose.position;
ins_orientation = data.ins.pose.orientation;

% Position stuff
ins_N = ins_pos(1,:);
ins_E = ins_pos(2,:);
ins_D = ins_pos(3,:);

% Plot Position
figure(1)
plot3(ins_N, ins_E, -ins_D);
hold on
% comet3(ins_N, ins_E, -ins_D);

% zeros out ins measurements
ins_zeroN = mean(ins_pos(1,1:10));
ins_zeroE = mean(ins_pos(2,1:10));
ins_zeroD = mean(ins_pos(3,1:10));

ins_zN = ins_N - ins_zeroN;
ins_zE = ins_E - ins_zeroE;
ins_zD = ins_D - ins_zeroD;

ins_pos(1,:) = ins_zN;
ins_pos(2,:) = ins_zE;
ins_pos(3,:) = ins_zD;

plot3(ins_zN, ins_zE, -ins_zD);

%%
% Aruco Data
aruco_t = data.aruco.marker_center.t;
aruco_pix = data.aruco.marker_center.point;

%% Match time steps and interpolate
indx = zeros(length(aruco_pix));
insNED = zeros(3, length(aruco_pix));
insEuler = zeros(3,length(aruco_pix));

for i=1:length(aruco_pix)
    [c, index] = min(abs(ins_t-aruco_t(i)));
    indx(i) = index;
    % mark new arrays of ins data
%     insNED(:,i) = ins_pos(:,index);
%     insNED(3,i) = ins_zD(:,index);
%     insEuler(:,i) = quat2euler(ins_orientation(:,index));
% end

    % Lets try to interpolate
    if (ins_t(index)-aruco_t(i)) < 0
    % then step forward one
        inspos1 = ins_pos(:,index);
        inspos2 = ins_pos(:,index+1);
        inseuler1 = quat2euler(ins_orientation(:,index));
        inseuler2 = quat2euler(ins_orientation(:,index+1));
        time_diff = ins_t(index+1) - ins_t(index);
        diff1 = ins_t(index+1) - aruco_t(i);
        inspos = inspos1*diff1/time_diff + inspos2*(time_diff-diff1)/time_diff;
        inseuler = inseuler1*diff1/time_diff + inseuler2*(time_diff-diff1)/time_diff;
        
        insNED(:,i) = inspos;
%         insNED(3,i) = ins_zD(:,index);
        insEuler(:,i) = inseuler;

    elseif (ins_t(index)-aruco_t(i)) > 0
    % then step back one
        inspos1 = ins_pos(:,index-1);
        inspos2 = ins_pos(:,index);
        inseuler1 = quat2euler(ins_orientation(:,index-1));
        inseuler2 = quat2euler(ins_orientation(:,index));
        time_diff = ins_t(index) - ins_t(index-1);
        diff1 = ins_t(index) - aruco_t(i);
        inspos = inspos1*diff1/time_diff + inspos2*(time_diff-diff1)/time_diff;
        inseuler = inseuler1*diff1/time_diff + inseuler2*(time_diff-diff1)/time_diff;
        
        insNED(:,i) = inspos;
%         insNED(3,i) = ins_zD(:,index);
        insEuler(:,i) = inseuler;
    else
    % no interpolation
        insNED(:,i) = ins_pos(:,index);
%         insNED(3,i) = ins_zD(:,index);
        insEuler(:,i) = quat2euler(ins_orientation(:,index));
    end
end


%% Geolocation
addpath('../Optimization/Offline')

fx = 566.669077;
fy = 566.403872;
ox = 399.829964;
oy = 297.588710;

geolocations = zeros(3, length(aruco_pix));

for i=1:length(aruco_pix)
    n = insNED(1,i);
    e = insNED(2,i);
    d = insNED(3,i);
    phi = insEuler(1,i);    
    theta = insEuler(2,i);
    psi = insEuler(3,i);
    px = aruco_pix(1,i);
    py = aruco_pix(2,i);

    phi_g = 0;
    theta_g = 0;
    psi_g = 0;
    x_g = 0.0;
    y = 0;
    z = 0.0;

    geolocations(:,i) = geolocation(n, e, d, phi, theta, psi, phi_g, theta_g, psi_g, x_g, y, z, px, py, fx, ox, fy, oy);
end

% Plot geolocations
for i=1:length(geolocations(1,:))
    figure(2)
    plot(geolocations(2,i), geolocations(1,i), 'o')
    hold on 
    plot(insNED(2,i), insNED(1,i), 'ro')
    axis equal
    title('raw geolocations')
%     pause;
end

%% Optimization

addpath('INSOptimization')

% Variables
n = insNED(1,:);
e = insNED(2,:);
d = insNED(3,:);
phi = insEuler(1,:);    
theta = insEuler(2,:);
psi = insEuler(3,:);
px = aruco_pix(1,:);
py = aruco_pix(2,:);

% phi_g = 0;
% theta_g = 0;
psi_g = 0;
x = 0;
y = 0;
z = 0;

fx = 566.669077;
fy = 566.403872;
ox = 399.829964;
oy = 297.588710;

% Function
f = @(x)INSvariance_func(x, n, e, d, phi, theta, psi, psi_g, x, y, z, px, py, fx, ox, fy, oy);

% Inputs to Optimizer
x0 = [0.0,  0.0];

A = [1, 0; ...
    0, 1; ... 
    -1, 0; ...
    0, -1];


Max = deg2rad(45);

b = Max*ones(4,1);

% Optimizer
% OPTIONS = optimoptions('fmincon','Algorithm','interior-point', 'HessianApproximation', 'bfgs')
% [x, variance] = fmincon(f, x0, A, b, [],[],-Inf,Inf, [],OPTIONS);
% [x, variance] = fmincon(f, x0, A, b);

x1 = [0, 0, 0, 0, 0, 0];

[x, variance] = fminunc(f, x1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Display resulting optimization %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('roll: '); disp(x(1));
disp('pitch: '); disp(x(2));

orig_var = f([0,0,0,0,0,0]);

%% Post Opt Geolocations

for i=1:length(aruco_pix)
    n = insNED(1,i);
    e = insNED(2,i);
    d = insNED(3,i);
    phi = insEuler(1,i);    
    theta = insEuler(2,i);
    psi = insEuler(3,i);
    px = aruco_pix(1,i);
    py = aruco_pix(2,i);

    phi_g = x(1);
    theta_g = x(2);
    psi_g = x(3);
    x_g = x(4);
    y = x(5);
    z = x(6);

    geolocations(:,i) = geolocation(n, e, d, phi, theta, psi, phi_g, theta_g, psi_g, x_g, y, z, px, py, fx, ox, fy, oy);
end

% Plot geolocations
for i=1:length(geolocations(1,:))
    figure(3)
    plot(geolocations(2,i), geolocations(1,i), 'o')
    hold on 
    plot3(insNED(2,i), insNED(1,i), -insNED(3,i), 'ro')
%     axis equal
    title('Optimized geolocations')
%     pause;
end
