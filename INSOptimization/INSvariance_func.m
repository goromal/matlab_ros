function variance = INSvariance_func(x, n, e, d, phi, theta, psi, psi_g, x_g, y_g, z_g, px, py, fx, ox, fy, oy )
%variance_func outputs variance of computed geolocations
%   Detailed explanation goes here


%init geolocations array
% geolocations = zeros(length(data),3);
% errors = zeros(length(data),1);

% % Args for geolocation
% n = p_uav(:,1);
% e = p_uav(:,2);
% d = p_uav(:,3);
% 
% phi = copter_angles(:,1);
% theta = copter_angles(:,2);
% psi = copter_angles(:,3);

phi_g = x(1);
theta_g = x(2);
psi_g = x(3);

x_g = x(4);
y_g = x(5);
z_g = x(6);

% px = pixels(:,1);
% py = pixels(:,2);
% 
% fx = 1691.9185737413309;
% fy = 1691.9185737413309;
% ox = 644.5;
% oy = 482.5;

% Compute geolocations
for i=1:1:length(n)
    if 1%(abs(phi(i)) < 0.01 && abs(theta(i)) < 0.01)
        geolocations(i,:) = geolocation(n(i), e(i), d(i), phi(i), theta(i), psi(i), phi_g, theta_g, psi_g, x_g, y_g, z_g, px(i), py(i), fx, ox, fy, oy);
        errors(i) = sqrt(geolocations(i,1)^2 + geolocations(i,2)^2);
    end
end

% Compute variance
variance = sum(errors)/length(errors);



end

