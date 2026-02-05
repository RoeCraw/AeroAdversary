function gpsXYZ = init(phi, HPA, VPA, fs, trueLLA) % OUTPUTS XYZ
    % INIT Initialize GPS sensor and get measurements
    % Inputs:
    %   phi - Decay factor
    %   HPA - Horizontal Position Accuracy  
    %   VPA - Vertical Position Accuracy
    %   fs - Sample rate
    %   trueLLA - True LLA positions (N×3)
    %
    % Outputs:
    %   gpsXYZ = [xGPS, yGPS, zGPS] - GPS measurements in NED coordinates
    
    pts = size(trueLLA, 1);
    refLLA = trueLLA(1, :); % Reference LLA for NED conversion
    
    % Create GPS sensor
    GPS = gpsSensor( ...
        "PositionInputFormat", "Geodetic", ...
        "HorizontalPositionAccuracy", sqrt(HPA), ...
        "VerticalPositionAccuracy", sqrt(VPA), ...
        "SampleRate", fs, ...
        "DecayFactor", phi);
    
    % Preallocate
    gpsLLA = zeros(pts, 3);
    
    % Get GPS measurements
    for i = 1:pts
        gpsLLA(i, :) = GPS(trueLLA(i, :), [0,0,0]); % no velo
    end
    
    % Convert to NED
    gpsNED = lla2ned(gpsLLA, refLLA, 'flat');
    
    % NED to XYZ
    xGPS = gpsNED(:, 2);  % East = X
    yGPS = gpsNED(:, 1);  % North = Y  
    zGPS = -gpsNED(:, 3); % Down = Up (negative z)

    gpsXYZ = [xGPS,yGPS,zGPS];
end