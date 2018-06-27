%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% This file takes 
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% User Options %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Choose the data file to run
% datafile = '3_alt_0_1_pitch_raw.csv';
% datafile = '3_alt_no_offset_raw.csv';

%%%%%%% The six isolated offsets %%%%%%%%
% They all seem to work %
% datafile = '0_1_x_raw.csv';
% datafile = '0_1_y_raw.csv';
% datafile = '0_1_z_raw.csv';
% datafile = '0_1_r_raw.csv';
% datafile = '0_n1_p_raw.csv';
% datafile = '0_1_yaw_raw.csv';

%%%%%%%%% Jesses Nasty Files %%%%%%%%%%%%
datafile = '0_05x_n0_02y_0_01z--n0_01r_n0_02p_0_05y_raw.csv';
% datafile = '0_02x_0_01y_0_03z--0_01r_n0_02p_n0_05y_raw.csv';
% datafile = '0_05-0_01-0_1--0_1-0_07-0_03_raw.csv';
% datafile = 'n0_05x_n0_01y_n0_1z--n0_1r_n0_07p_n0_03y_raw.csv';

% Do you want to make pre/post optimization plots?
plots = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Optimization %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Function
f = @(x)variance_func(x, datafile);

% Inputs to Optimizer
x0 = [0.0,  0.0, 0.0, 0.0, 0.0, 0.0];

A = [1, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0;
    -1, 0, 0, 0, 0, 0;
    0, -1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, -1, 0, 0, 0];

Max = deg2rad(45);

b = Max*ones(6,1);

% Optimizer
% OPTIONS = optimoptions('fmincon','Algorithm','interior-point', 'HessianApproximation', 'bfgs')
% [x, variance] = fmincon(f, x0, A, b, [],[],-Inf,Inf, [],OPTIONS);
[x, variance] = fmincon(f, x0, A, b);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Display resulting optimization %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('roll: '); disp(x(1));
disp('pitch: '); disp(x(2));
disp('yaw: '); disp(x(3));
disp('x: '); disp(x(4));
disp('y: '); disp(x(5));
disp('z: '); disp(x(6));

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Make Plots %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if plots
   
    % Get geolocations for pre and post optimizations
    [preGeos, copter, prevariance] = getGeoPoints(x0, datafile, false);
    [postGeos, copter, postvariance] = getGeoPoints(x, datafile, true);
    
    % Make plot for preOptimization
    figure()
    plot(preGeos(:,2), preGeos(:,1),'bo',0,0,'k*');
    hold on
    plot(copter(:,2), copter(:,1), 'gs');
    xlabel('East (m)');
    ylabel('North (m)');
    title('Geolocations for PreOptimization');
    axis equal
    
    % Make plot for postOptimization
    figure()
    plot(postGeos(:,2), postGeos(:,1),'bo',0,0,'k*');
    hold on
    plot(copter(:,2), copter(:,1), 'gs');
    xlabel('East (m)');
    ylabel('North (m)');
    title('Geolocations for PostOptimization');
    axis equal
    
end
