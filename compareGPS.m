clear all;
close all;
clc;

%% Create Trajectory + Data conversion

pts = 100;
t = linspace(0, 10, pts); % Time vector

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
plot(xTrue,yTrue,'b.', MarkerSize=10,DisplayName='True Data')
hold on;
legend();
xlabel("X position (m)")
ylabel("Y position (m)")
title("True Path with FOGM GPS Sensor Noise ");
axis equal;
grid;
plot(xGPS,yGPS,'r.',MarkerSize=10, DisplayName='GPS Data')

disp("GPS X coordinate mean: " + mean(xGPS,1))
disp("GPS Y coordinate mean: " + mean(yGPS,1))
disp("GPS X coordinate variance: " + var(xGPS,1))
disp("GPS Y coordinate variance: " + var(yGPS,1))