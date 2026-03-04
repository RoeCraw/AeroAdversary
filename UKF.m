close all;
clear all;
clc;
%% Trajectory

pts = 100;
t = linspace(0,10, pts);
dt = t(2) - t(1);
xTrue = t*2;
yTrue = 5*t - 0.5*t.^2;
vxTrue = ones(1, pts) * 2;
vyTrue = 5 - t;
stateTrue = [xTrue; yTrue; vxTrue; vyTrue];

% figure(1);
% plot(xTrue, yTrue, 'k', LineWidth=2);
% xlabel('X position (m)');
% ylabel('Y position (m)');
% legend('xTrue', 'yTrue');
% title('True State Trajectories');
% grid on;
%% Unscented Transformation
init = [xTrue(1), yTrue(1), 2, 5]; % x, y, vx, vy
Q = diag([0.1,0.1,0.5,0.5]);
R = diag([0.1,0.01]);

L = length(init);
alpha = 1e-3;
kappa = 0;
lam = alpha^2 * (L + kappa) - L;
beta = 2;

% Weights
W_m = [lam / (L + lam)];
W_c = [W_m(1) + (1 - alpha^2 + beta)];
for j = 1:2*L
    W_m(:, end + 1) = 1/(2*(L + lam));
    W_c(:, end + 1) = 1/(2*(L + lam));
end

%% Main
state = init(:);
cv = eye(L) * 0.1;
history = [];
measurements = [];

for i = 1:pts
    % 1) Prediction
    sig = sigma(state, cv, L, lam);
    
    S_pred = [];
    for j = 1:size(sig,2)
        s = sig(:, j);
        S_pred(:, j) = f(s, dt);
    end

    mean_pred = zeros(L,1);
    for j = 1:(2*L + 1)
        mean_pred = mean_pred + W_m(j) * S_pred(:,j);
    end

    cv_pred = zeros(L,L);
    for j = 1:(2*L + 1)
        diff = S_pred(:,j) - mean_pred;
        cv_pred = cv_pred + W_c(j) * (diff * diff');
    end
    cv_pred = cv_pred + Q;

    % 2) Update Step (Predicted Measurement - heart of the UKF)
    S_meas = [];
    for j = 1:size(S_pred, 2) % run predicted sigma points through measurement function
        s = S_pred(:, j);
        S_meas(:, j) = h(s);
    end

    % Predicted measurement mean (range linear, bearing circular)
    pred_meas = zeros(2,1);
    for j = 1:(2*L + 1)
        pred_meas(1) = pred_meas(1) + W_m(j) * S_meas(1, j);
    end
    pred_meas(2) = angle_mean(S_meas(2, :), W_m);

    % Predicted measurement covariance
    meas_cv = R;
    for j = 1:(2*L + 1)
        diff_meas = S_meas(:, j) - pred_meas;
        diff_meas(2) = wrap_angle(diff_meas(2));
        meas_cv = meas_cv + W_c(j) * (diff_meas * diff_meas');
    end

    % Cross covariance
    cross_cv = zeros(4, 2);
    for j = 1:(2*L + 1)
        diff_state = S_pred(:, j) - mean_pred;
        diff_meas = S_meas(:, j) - pred_meas;
        diff_meas(2) = wrap_angle(diff_meas(2));
        cross_cv = cross_cv + W_c(j) * (diff_state * diff_meas');
    end

    % Correction Step (Predicted Measurement vs Actual Measurement +
    % Kalman Gain
    
    K = cross_cv / meas_cv;

    z_true = h(stateTrue(:, i));
    % Generate noisy measurement
    z_meas = z_true + sqrtm(R) * randn(2, 1);
    z_meas(2) = wrap_angle(z_meas(2));
    measurements(:, i) = z_meas; 

    wrap = z_meas - pred_meas;
    wrap(2) = wrap_angle(wrap(2));

    state = mean_pred + K * wrap;
    cv = cv_pred - K * meas_cv * K';

    history(:, i) = state;

end

ukf_estimates = history;
meas_x = measurements(1,:) .* cos(measurements(2,:));
meas_y = measurements(1,:) .* sin(measurements(2,:));

figure(2)
plot(xTrue, yTrue, 'k', LineWidth=2, DisplayName='True Trajectory')
hold on;
plot(meas_x, meas_y, 'r.', DisplayName='Measurements')
plot(ukf_estimates(1, :), ukf_estimates(2, :), 'b.', DisplayName='UKF Estimates')  
legend show;
xlim
xlabel('X position (m)');
ylabel('Y position (m)');
title('UKF Trajectory Estimates vs True Trajectory');
grid on;
xlim([-1, 20]); 
ylim([-1, 15]); 
hold off;


%% Functions
function out = f(state, dt) % state transition
    x = state(1);
    y = state(2);
    vx = state(3);
    vy = state(4); 

    new_x = x + vx * dt;
    new_y = y + vy * dt;
    new_vx = vx;
    new_vy = vy - 1.0 * dt;
    out = [new_x; new_y; new_vx; new_vy];
end

function out = h(state) % measurement
    x = state(1);
    y = state(2);
    range_ = sqrt(x^2 + y^2);
    bearing = atan2(y, x);
    out = [range_; bearing];
end

function X = sigma(mean, cov, L, lam) % sigm point generator
    mean = mean(:);
    S = sqrtm((L+lam)*cov);
    X = zeros(L, 2*L + 1);
    X(:, 1) = mean;
    for i = 1:L
        X(:, i + 1) = mean + S(:, i);
        X(:, i + 1 + L) = mean - S(:, i);
    end
end

function out = wrap_angle(a) % mod div for angles [-pi to pi]
    out = mod((a + pi),(2*pi)) - pi;
end

function out = angle_mean(angles, weights)
    sin_sum = sum(weights .* sin(angles));
    cos_sum = sum(weights .* cos(angles));  
    out = atan2(sin_sum, cos_sum);
end


