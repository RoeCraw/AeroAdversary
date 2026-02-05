clear all;
close all;
clc;

%% Create Trajectory + Data conversion

pts = 100;
t = linspace(0, 10, pts); % Time vector


% slight curve:
% xTrue = [];
% yTrue = [];
% zTrue = [];
% for i = 1:pts
%    xTrue(i) = 50 * t(i) - 0.2 * i * t(i);
%    yTrue(i) = 20 * t(i);
%    zTrue = zeros(1, pts);
% end

xTrue = 50 * t;
yTrue = 20 * t;
zTrue = zeros(1, pts);

refLLA = [45.0, -45.0, 0]; % fake location for reference

trueData = [xTrue;yTrue]';

trueNED = [yTrue',xTrue',-zTrue']; % xyz to NED (north, east, down)
trueLLA = ned2lla(trueNED,refLLA, "flat");


%% GPS Sensor + Parameters

phi = 0.90;
horizontalPositionAccuracy = 100;
verticalPositionAccuracy = 100;
fs = pts/10;


N = 10;
sat = zeros(pts, 3, N); % tensor

for i = 1:N
    sat(:,:,i) = init(phi, horizontalPositionAccuracy, verticalPositionAccuracy, fs, trueLLA);
end

%%

% figure(1)
% plot(xTrue,yTrue,'k.', MarkerSize=10,DisplayName='True Data')
% hold on;
% 
% xlabel("X position (m)")
% ylabel("Y position (m)")
% title("True Path with FOGM GPS Sensor Noise ");
% axis equal;
% grid;
% 
% 
% for i = 1:N
%     plot(sat(:,1,i),sat(:,2,i), ...
%         Marker = '.', ...
%         LineStyle='none',...
%         MarkerSize=10, ...
%         DisplayName = ['GPS Data ' num2str(i)] );
% end
% 
% legend('show');

%%
fusedPos = zeros(pts,2);

for i = 1:pts
    x = 0;
    y = 0;
    for j = 1:N
        x = x + sat(i,1,j);
        y = y + sat(i,2,j);
    end
    x = x / N; % Average x position
    y = y / N; % Average y position

    fusedPos(i,:) = [x y];
end

figure(1)
plot(xTrue,yTrue,'k-', LineWidth=2,DisplayName='True Data')
hold on;
xlabel("X position (m)")
ylabel("Y position (m)")
title("True Path with FOGM GPS Sensor Noise ");
axis equal;
grid;
plot(fusedPos(:,1),fusedPos(:,2), 'r-', LineWidth=2, ...
    DisplayName = "Fused Data");

for i = 1:N
    plot(sat(:,1,i),sat(:,2,i), ...
        Marker = '.', ...
        LineStyle='none',...
        MarkerSize=5);
end

legend({'True Data', 'Fused Data'});

%% Kalman Filter

dt = t(2) - t(1);
initialState = [fusedPos(1,1); 0; fusedPos(1,2); 0]; % [x,vx,y,vy]
KF = trackingKF('MotionModel', '2D Constant Velocity', 'State', initialState);

kalmanPos = zeros(pts, 2);
for k = 1:pts
    predict(KF, dt); % predict -> outputs predicted state xpred and the predicted state estimation error covariance
    correctedState = correct(KF, fusedPos(k,:)); % update
    kalmanPos(k,:) = [correctedState(1), correctedState(3)]; % Correct x and y only (no velo)
end

plot(kalmanPos(:,1), kalmanPos(:,2), 'b-', LineWidth=2, DisplayName='Kalman Filter');