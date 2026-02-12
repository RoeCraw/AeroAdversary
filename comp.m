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
horizontalPositionAccuracy = 100;
verticalPositionAccuracy = 100;
velocityAccuracy = 1;
fs = pts/10;

GPS = gpsSensor( ...
    "PositionInputFormat","Geodetic", ...
    "HorizontalPositionAccuracy",sqrt(horizontalPositionAccuracy), ...
    "VerticalPositionAccuracy",sqrt(verticalPositionAccuracy), ...
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


figure(1)
plot(xTrue,yTrue,'b',LineWidth=2,DisplayName='True Data')
hold on;
legend();
xlabel("X position (m)")
ylabel("Y position (m)")
title("True Path with FOGM GPS Sensor Noise ");
axis equal;
grid;
plot(xGPS,yGPS, 'r', LineWidth=2, DisplayName='GPS Data')

% disp("GPS X coordinate mean: " + mean(xGPS,1))
% disp("GPS Y coordinate mean: " + mean(yGPS,1))
% disp("GPS X coordinate variance: " + var(xGPS,1))
% disp("GPS Y coordinate variance: " + var(yGPS,1))

%% Testing
%{
A matrix - The state transition matrix. This matrix describes how the
system state evolves over time. For our situation, we use a matrix full of
ones and zeros as well as dt in cases where we are not measuring the
velocity

P matrix - The error covariance matrix at time k (mxn). This is updated
throughout the predict and the correct steps.

H matrix - The noiseless state-to-measurement matrix. This matrix is used
to convert the system state estimate from the state space to the
measurement space. In all the cases I've seen, this is zeros and ones, but
this is made up of differential equations for the EKF. If you are measuring
positon and not velocity, you were have [1,0] (if the system is 1D)

Q matrix - The Q matrix represents the uncertainty of the system model. This
matrix is known as the process noise covariance. This determines how much
the model's predictions diverge from the true system dynamics due to
simplifications, unknown external forces, or disturbances. 

R matrix - The measurement covariance matrix (error in sensors)

I matrix -  Identity matrix

%}

A = [1,dt,0,0; 
     0,1,0,0;
     0,0,1,dt;
     0,0,0,1];

P = [1,0,0,0; 
     0,1,0,0;
     0,0,1,0;
     0,0,0,1];

H = [1,0,0,0; 
     0,0,1,0];

Q = zeros(4,4);

R = [5,0; 
     0,5]; % +/- 5 meters

I = eye(4);

state = [xGPS(1); 0; yGPS(1); 0];
state_history = zeros(pts, 4);

for i = 1:pts
    % Predict
    state_predict = A * state;
    P_predict = A * P * A' + Q;
    
    % Update
    measurement = [xGPS(i); yGPS(i)];
    K = P_predict * H' * inv(H * P_predict * H' + R);
    state = state_predict + K * (measurement - H * state_predict);
    P = (I - K * H) * P_predict;

    % if mod(i,5) == 0
    %     fprintf('Kalman Gain = \n');
    %     disp(K')
    % end

    state_history(i, :) = state';

end

plot(state_history(:,1), state_history(:,3), 'm',LineWidth=4, DisplayName='KF Position (ME)');


%% Kalman Filter - Sensor Fusion and Tracking Toolbox
gpsData = [xGPS, yGPS];

Q = zeros(2);

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

plot(kalmanPos(:,1), kalmanPos(:,2), 'g',LineWidth=2, DisplayName='KF Position (SF)');
hold off;

%% Compare
% Root mean square error (RMSE) which compares the euclidean distance error
% aka L2 norm

rmseGPS = sqrt((xGPS - xTrue).^2 + (yGPS - yTrue).^2);
rmseKF_Matlab = sqrt((kalmanPos(:,1) - xTrue).^2 + (kalmanPos(:,2) - yTrue).^2);
rmseKF_Me = sqrt((state_history(:,1) - xTrue).^2 + (state_history(:,3) - yTrue).^2);

figure(2);
plot(t,rmseGPS, 'k-',LineWidth=2, DisplayName='GPS Error')
hold on;
legend('show');
grid();
ylabel('Error (m)');
xlabel('Time (s)')
plot(t,rmseKF_Matlab, 'b-',LineWidth=4, DisplayName='Matlab Error')
plot(t,rmseKF_Me, 'r-',LineWidth=2, DisplayName='My Error')
title('Root Mean Square Error of Kalman Filter Methods')

fprintf('RMSE for GPS: %.2f m\n', mean(rmseGPS));
fprintf('RMSE for Kalman Filter (Matlab): %.2f m\n', mean(rmseKF_Matlab));
fprintf('RMSE for Kalman Filter (My Implementation): %.2f m\n', mean(rmseKF_Me));


