clear all;
close all;
clc;

%% Create Trajectory and GPS Data 

pts = 100;
t = linspace(0, 10, pts); % Time vector
dt = t(2) - t(1);

xTrue = (50 * t)';
yTrue = (20 * t)';
zTrue = zeros(1, pts)';

refLLA = [45.0, -45.0, 0]; % fake location for reference

trueData = [xTrue;yTrue];

trueNED = [yTrue,xTrue,-zTrue]; % xyz to NED (north, east, down)
trueLLA = ned2lla(trueNED,refLLA, "flat");

% GPS Sensor + Parameters

phi = 0.90;
horizontalPositionAccuracy = 5;
verticalPositionAccuracy = 5;
velocityAccuracy = 1;
fs = pts/10;

GPS = gpsSensor( ...
    "PositionInputFormat","Geodetic", ...
    "HorizontalPositionAccuracy",horizontalPositionAccuracy, ...
    "VerticalPositionAccuracy",verticalPositionAccuracy, ...
    "SampleRate",fs, ...
    "DecayFactor", phi, ...
    "RandomStream", "mt19937ar with seed", ...
    "Seed", 11);

for i = 1:pts
    gpsLLA(i, :) = GPS(trueLLA(i, :), [0,0,0]); % no velo
end

gpsNED = lla2ned(gpsLLA, refLLA, 'flat');
xGPS = gpsNED(:,2);
yGPS = gpsNED(:,1);
zGPS = -gpsNED(:,3);

error = 5;

for i = 1:pts
    xGPS(i) = xGPS(i) + error*(t(i)/60);
    yGPS(i) = yGPS(i) + error*(t(i)/60);
end

figure(1)
plot(xTrue,yTrue,LineWidth=2,DisplayName='True Data')
hold on;
axis equal;
plot(xGPS,yGPS, LineWidth=2, DisplayName='GPS Data')


%% Kalman Filter - Sensor Fusion and Tracking Toolbox
gpsData = [xGPS, yGPS];

Qvals = [100,10,1,0.1,0.01,0.001];
%Qvals = [100,0.001];
%Qvals = 0:0.5:10;
rmse = zeros(pts, length(Qvals));

for i = 1:length(Qvals)
    Q = Qvals(i);
    
    R = [5,0;
        0,5];
    
    H = [1,0,0,0; 
         0,0,1,0];
    
    initialState = [xGPS(1); 0; yGPS(1); 0]; % [x,vx,y,vy]
    KF = trackingKF('MotionModel', '2D Constant Velocity', ...
        'State', initialState, ...
        'ProcessNoise', Q, ...
        'MeasurementModel', H,...
        'MeasurementNoise',R);
    
    kalmanPos = zeros(pts, 2);
    for k = 1:pts
        predict(KF, dt); % predict -> outputs predicted state xpred and the predicted state estimation error covariance
        correctedState = correct(KF, gpsData(k,:)); % update
        kalmanPos(k,:) = [correctedState(1), correctedState(3)]; % Correct x and y only (no velo)
    end
    
    plot(kalmanPos(:,1), kalmanPos(:,2), 'LineWidth', 2,'DisplayName', sprintf('Q value = %.5f', Qvals(i)));
    rmse(:,i) = sqrt((kalmanPos(:,1) - xTrue).^2 + (kalmanPos(:,2) - yTrue).^2); % see if there is a way to non-dimensionalize it
    fprintf('RMSE = %.4f ; Q = %.3f\n', mean(rmse(:,i)), Qvals(i));
end

legend('show');
title('Kalman Filter Results with Varying Process Noise');
grid("on")
xlabel('x position (m)')
ylabel('y position (m)')


%%

figure(2)
for i = 1:length(Qvals)
    plot(t, rmse(:,i), 'LineWidth', 2,'DisplayName', sprintf('Q value = %.4f', Qvals(i)));
    hold on;
end
legend('show');
title('Kalman Filter Results with Varying Process Noise');
grid("on")
xlabel('Time (s)')
ylabel('RMSE (m)')
