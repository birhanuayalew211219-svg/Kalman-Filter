% main_quat_EKF_attitude.m
% Quaternion-based EKF for Attitude Estimation with Star Sensor and Linearized Jacobian

clc; clear; close all;
randn('seed', 12345)
rand('seed', 12345)

%% Constants
mu = 398600;               % Earth's gravitational parameter [km^3/s^2]
radius_earth = 6371;       % Earth's radius in km
T = 1;                     % Sampling interval [s]
N = 10000;                 % Simulation steps

%% Initial Conditions
pos0   = [7000; 0; 0];
vel0   = [0; 7.5; 0];
q0     = [1; 0; 0; 0];
omega0 = [0; 0; 0.001];
x0     = [pos0; vel0; q0; omega0];

%% Noise Covariances
Q      = 1e-6 * eye(13);   % Process noise
R_gyro = 1e-4 * eye(3);    % Gyro measurement noise
R_star = 1e-6 * eye(4);    % Star sensor noise
R_pos  = 1e-7 * eye(3);    % Position measurement noise

%% Initialization
x_true = zeros(13, N);
x_est  = zeros(13, N);
P      = 1e-2 * eye(13);

x_true(:,1) = x0;
x_est(:,1)  = x0;
x_est(7:10,1)   = x_est(7:10,1)   + 1e-4*randn(4,1);
x_est(11:13,1)  = x_est(11:13,1)  + 1e-4*randn(3,1);

%% Main EKF Loop
for k = 2:N
    % True propagation
    x_true(:,k) = rk4(@attitude_dynamics, x_true(:,k-1), T);

    % Predicted state
    x_pred = rk4(@attitude_dynamics, x_est(:,k-1), T);

    % Linearized dynamics
    F = compute_jacobian(x_est(:,k-1));
    P = F*P*F' + Q;

    %% Gyro Update
    z_gyro   = x_true(11:13,k) + sqrt(R_gyro)*randn(3,1);
    H_gyro   = zeros(3,13); H_gyro(:,11:13) = eye(3);
    K_gyro   = P*H_gyro' / (H_gyro*P*H_gyro' + R_gyro);
    x_pred   = x_pred + K_gyro*(z_gyro - H_gyro*x_pred);
    P        = (eye(13) - K_gyro*H_gyro)*P;

    %% Position Update with non-Gaussian Z impulse
    z_pos = x_true(1:3,k) + 0.05 *sqrt(R_pos) * randn(3,1);
    if rand < 0.05
        z_pos(3) = z_pos(3) + 0.05 * randn();
    end
    H_pos = zeros(3,13); H_pos(:,1:3) = eye(3);
    K_pos = P * H_pos' / (H_pos * P * H_pos' + R_pos);
    x_pred = x_pred + K_pos * (z_pos - H_pos * x_pred);
    P      = (eye(13) - K_pos * H_pos) * P;

    %% Star Sensor Update
    z_star = x_true(7:10,k) + sqrt(R_star)*randn(4,1);
    H_star = zeros(4,13); H_star(:,7:10) = eye(4);
    K_star = P*H_star' / (H_star*P*H_star' + R_star);
    x_est(:,k) = x_pred + K_star*(z_star - H_star*x_pred);
    P          = (eye(13) - K_star*H_star)*P;

    % Normalize quaternion
    x_est(7:10,k) = x_est(7:10,k) / norm(x_est(7:10,k));
end

%% Quaternion Angular Error (in degrees)
q_error_angle = zeros(1, N);
for k = 1:N
    q_true_k = x_true(7:10, k);
    q_est_k  = x_est(7:10, k);

    q_true_k = q_true_k / norm(q_true_k);
    q_est_k  = q_est_k  / norm(q_est_k);

    angle_rad         = 2 * acos(abs(dot(q_true_k, q_est_k)));
    q_error_angle(k)  = rad2deg(angle_rad);
end

%% Euler angles [yaw; pitch; roll]
eul_true = zeros(3,N);
eul_est  = zeros(3,N);
eul_err  = zeros(3,N);   % error in degrees

for k = 1:N
    qt = x_true(7:10,k);
    qe = x_est(7:10,k);

    eul_true(:,k) = quat2eulZYX(qt);
    eul_est(:,k)  = quat2eulZYX(qe);

    eul_err(:,k)  = rad2deg(eul_est(:,k) - eul_true(:,k));
end

figure;
titles = {'Yaw Error', 'Pitch Error', 'Roll Error'};
for i = 1:3
    subplot(3,1,i);
    plot(1:N, eul_err(i,:), 'b', 'LineWidth', 1.5);
    ylabel('[deg]'); grid on;
    title(titles{i});
    if i == 3, xlabel('Time Step'); end
end

% Euler RMSE
eul_rmse = sqrt(mean(eul_err.^2, 2));
fprintf('\nEuler Angle RMSEs (degrees):\n');
fprintf('Yaw   RMSE: %.4f°\n',  eul_rmse(1));
fprintf('Pitch RMSE: %.4f°\n',  eul_rmse(2));
fprintf('Roll  RMSE: %.4f°\n',  eul_rmse(3));

%% 3D Orbit + Attitude Animation + Video Recording
video_filename = 'orbit_attitude.avi';
v = VideoWriter(video_filename);
v.FrameRate = 15;
open(v);

figure;
axis equal;
xlabel('X [km]'); ylabel('Y [km]'); zlabel('Z [km]');
title('Orbit + Attitude Animation');
grid on; hold on; view(45, 25);

plot3(x_true(1,:), x_true(2,:), x_true(3,:), 'k:', 'LineWidth', 0.5);

step  = 70;     % frame skip
scale = 500;    % body axis length

for k = 1:step:N
    clf;

    % Earth with texture
    [xx, yy, zz] = sphere(180); 
    load topo; topo = flipud(topo);
    surf(radius_earth * xx, radius_earth * yy, radius_earth * zz, ...
        'FaceColor', 'texturemap', 'EdgeColor', 'none', ...
        'CData', topo, 'FaceLighting', 'gouraud');
    light('Position', [-1 0 1], 'Style', 'infinite'); material shiny;

    hold on; grid on; axis equal;
    view(45, 25);
    xlabel('X [km]'); ylabel('Y [km]'); zlabel('Z [km]');
    title(sprintf('LEO satellite – Time Step: %d', k), 'FontSize', 13);

    pos = x_true(1:3,k);

    text(pos(1)+700, pos(2)+700, pos(3)+700, ...
         'LEO : 629 km', ...
         'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');

    plot3(x_true(1,1:k), x_true(2,1:k), x_true(3,1:k), 'r', 'LineWidth', 1.5);

    q_est_k = x_est(7:10,k) / norm(x_est(7:10,k));
    R = quat2dcm(q_est_k);

    quiver3(pos(1), pos(2), pos(3), scale*R(1,1), scale*R(2,1), scale*R(3,1), 'm', 'LineWidth', 2);
    quiver3(pos(1), pos(2), pos(3), scale*R(1,2), scale*R(2,2), scale*R(3,2), 'g', 'LineWidth', 2);
    quiver3(pos(1), pos(2), pos(3), scale*R(1,3), scale*R(2,3), scale*R(3,3), 'c', 'LineWidth', 2);

    plot3(pos(1), pos(2), pos(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y');

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);
fprintf('Video saved to: %s\n', video_filename);

%% 3D Attitude Visualization (body axes)
figure;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(135, 30);
grid on;
title('True vs Estimated Attitude (Body Axes)');
hold on;

% Inertial frame axes
quiver3(0,0,0,1,0,0,'k','LineWidth',1,'MaxHeadSize',0.5);
quiver3(0,0,0,0,1,0,'k','LineWidth',1,'MaxHeadSize',0.5);
quiver3(0,0,0,0,0,1,'k','LineWidth',1,'MaxHeadSize',0.5);

% Estimated axes (RGB)
bx_est = quiver3(0,0,0,1,0,0,'r','LineWidth',2);
by_est = quiver3(0,0,0,0,1,0,'g','LineWidth',2);
bz_est = quiver3(0,0,0,0,0,1,'b','LineWidth',2);

% True axes (dashed)
bx_true = quiver3(0,0,0,1,0,0,'--r','LineWidth',1.5);
by_true = quiver3(0,0,0,0,1,0,'--g','LineWidth',1.5);
bz_true = quiver3(0,0,0,0,0,1,'--b','LineWidth',1.5);

for k = 1:100:N
    q_est_k  = x_est(7:10,k) / norm(x_est(7:10,k));
    q_true_k = x_true(7:10,k) / norm(x_true(7:10,k));

    R_est  = quat2dcm(q_est_k);
    R_true = quat2dcm(q_true_k);

    set(bx_est, 'UData', R_est(1,1), 'VData', R_est(2,1), 'WData', R_est(3,1));
    set(by_est, 'UData', R_est(1,2), 'VData', R_est(2,2), 'WData', R_est(3,2));
    set(bz_est, 'UData', R_est(1,3), 'VData', R_est(2,3), 'WData', R_est(3,3));

    set(bx_true, 'UData', R_true(1,1), 'VData', R_true(2,1), 'WData', R_true(3,1));
    set(by_true, 'UData', R_true(1,2), 'VData', R_true(2,2), 'WData', R_true(3,2));
    set(bz_true, 'UData', R_true(1,3), 'VData', R_true(2,3), 'WData', R_true(3,3));

    drawnow;
end
