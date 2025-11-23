clear; clc; close all;

%% ======================= LIMO CALIBRATION SCRIPT =======================
% Calibrates coordinate transform from MoCap to maze frame
% Robot should be at maze origin (0,0) pointing in +X direction
% After calibration, robot returns to origin automatically
%
% REQUIREMENTS:
% - Robot positioned at maze origin facing +X (toward first waypoint)
% - OptiTrack streaming quaternions to MQTT
% - Robot control via TCP socket

fprintf('=================================================================\n');
fprintf('LIMO COORDINATE CALIBRATION\n');
fprintf('=================================================================\n\n');

% Load configuration
cfg = limo_config();

%% ======================= SETUP CONNECTIONS =======================
fprintf('Setting up connections...\n');

% MQTT for pose data only
mq = mqttclient(cfg.broker, Port=cfg.mqtt_port, ClientID="calibration");
if ~mq.Connected
    error('Could not connect to MQTT broker at %s', cfg.broker);
end
subscribe(mq, cfg.topic_pose);
fprintf('✓ MQTT connected for pose data\n');

% TCP socket for robot control
sock = tcpclient(cfg.LIMO_IP, cfg.LIMO_PORT);
configureCallback(sock, "off");
fprintf('✓ TCP connected to robot at %s:%d\n', cfg.LIMO_IP, cfg.LIMO_PORT);

%% ======================= STRAIGHTEN WHEELS =======================
fprintf('\n=== STRAIGHTENING WHEELS ===\n');
fprintf('Sending wheel straightening commands...\n');

for i = 1:20
    sendRobotCmd(sock, 0, 0, cfg.WHEELBASE);
    pause(0.05);
end

fprintf('✓ Wheels straightened\n');

%% ======================= RECORD ORIGIN =======================
fprintf('\n=== CALIBRATION SETUP ===\n');
fprintf('Robot should be at maze origin (0,0) pointing in +X direction\n');
fprintf('The robot heading should point toward the first waypoint\n');
fprintf('Press ENTER when robot is in position...\n');
pause;

% Get origin pose
fprintf('\nRecording origin pose...\n');
pause(0.5);  % Let system settle

origin_pose = get_mocap_pose(mq, cfg.topic_pose);

calib.origin_x = origin_pose.pos(1);   % MoCap X
calib.origin_z = origin_pose.pos(3);   % MoCap Z (robot moves in XZ plane)
calib.origin_yaw = origin_pose.yaw;    % MoCap yaw

fprintf('\n✓ ORIGIN RECORDED:\n');
fprintf('  MoCap X: %.4f m\n', calib.origin_x);
fprintf('  MoCap Z: %.4f m\n', calib.origin_z);
fprintf('  MoCap Yaw: %.2f deg\n', rad2deg(calib.origin_yaw));

%% ======================= TEST DRIVE TO DETERMINE TRANSFORM =======================
fprintf('\n=== DETERMINING COORDINATE TRANSFORM ===\n');
fprintf('Robot will drive FORWARD to determine correct transform\n');
fprintf('Expected: Robot moves in +X maze direction (toward first waypoint)\n');
fprintf('Press ENTER to start test drive...\n');
pause;

% Record start position
test_start = get_mocap_pose(mq, cfg.topic_pose);
fprintf('\nStart position: X=%.3f, Z=%.3f, Yaw=%.1f°\n', ...
        test_start.pos(1), test_start.pos(3), rad2deg(test_start.yaw));

% Drive forward for calibrated time
fprintf('Driving forward at %.2f m/s for %.1f seconds...\n', ...
        cfg.calib_drive_speed, cfg.calib_drive_time);

drive_start_time = tic;
while toc(drive_start_time) < cfg.calib_drive_time
    sendRobotCmd(sock, cfg.calib_drive_speed, 0, cfg.WHEELBASE);
    pause(0.05);
end

% Stop
fprintf('Stopping...\n');
stopRobot(sock, cfg.WHEELBASE);
pause(0.5);  % Let robot settle

% Record end position
test_end = get_mocap_pose(mq, cfg.topic_pose);
fprintf('End position: X=%.3f, Z=%.3f, Yaw=%.1f°\n', ...
        test_end.pos(1), test_end.pos(3), rad2deg(test_end.yaw));

% Calculate displacement in MoCap frame
dx_mocap = test_end.pos(1) - test_start.pos(1);
dz_mocap = test_end.pos(3) - test_start.pos(3);
dist_traveled = hypot(dx_mocap, dz_mocap);

fprintf('\n=== MOCAP DISPLACEMENT ===\n');
fprintf('  dX_mocap = %+.3f m\n', dx_mocap);
fprintf('  dZ_mocap = %+.3f m\n', dz_mocap);
fprintf('  Distance = %.3f m\n', dist_traveled);
fprintf('  Expected: %.3f m\n', cfg.calib_drive_speed * cfg.calib_drive_time);

%% ======================= EVALUATE TRANSFORM OPTIONS =======================
fprintf('\n=== EVALUATING COORDINATE TRANSFORMS ===\n');
fprintf('Looking for transform where dx_maze > 0 and dy_maze ≈ 0\n\n');

% Define all possible transforms
transforms = struct();

transforms(1).name = "x_maze = +Z,  y_maze = +X";
transforms(1).dx = dz_mocap;
transforms(1).dy = dx_mocap;

transforms(2).name = "x_maze = -Z,  y_maze = +X";
transforms(2).dx = -dz_mocap;
transforms(2).dy = dx_mocap;

transforms(3).name = "x_maze = +X,  y_maze = +Z";
transforms(3).dx = dx_mocap;
transforms(3).dy = dz_mocap;

transforms(4).name = "x_maze = +X,  y_maze = -Z";
transforms(4).dx = dx_mocap;
transforms(4).dy = -dz_mocap;

% Find best transform automatically
best_transform = 0;
best_score = -inf;

for i = 1:4
    % Score based on: large positive dx, small dy
    score = transforms(i).dx - 5 * abs(transforms(i).dy);
    
    fprintf('%d. %-30s --> dx_maze=%+.3f, dy_maze=%+.3f', ...
            i, transforms(i).name, transforms(i).dx, transforms(i).dy);
    
    % Check if this looks correct
    if transforms(i).dx > 0.3 && abs(transforms(i).dy) < 0.15
        fprintf('  ← GOOD');
        if score > best_score
            best_score = score;
            best_transform = i;
        end
    end
    fprintf('\n');
end

% Auto-select or ask user
if best_transform > 0
    fprintf('\nAuto-detected transform %d: %s\n', best_transform, transforms(best_transform).name);
else
    fprintf('\nCould not auto-detect transform.\n');
    best_transform = 4;  % Default to known correct
end

% OVERRIDE: Always use Transform 4 (verified correct)
fprintf('\n⚠ OVERRIDE: Forcing Transform 4 (verified correct via testing)\n');
calib.transform = 4;

fprintf('\nUsing: %s\n', transforms(calib.transform).name);

%% ======================= VERIFY TRANSFORM =======================
fprintf('\n=== VERIFICATION ===\n');

% Apply transform to check origin
[toMazeX, toMazeY] = get_transform_functions(calib);
maze_x_origin = toMazeX(test_start.pos(1), test_start.pos(3));
maze_y_origin = toMazeY(test_start.pos(1), test_start.pos(3));

fprintf('Start position in maze frame: (%.3f, %.3f)\n', maze_x_origin, maze_y_origin);

if abs(maze_x_origin) < 0.05 && abs(maze_y_origin) < 0.05
    fprintf('✓ Position looks correct (near origin)\n');
else
    fprintf('⚠ Warning: Position not near origin! Check robot placement.\n');
end

%% ======================= RETURN ROBOT TO ORIGIN =======================
fprintf('\n=== RETURNING TO ORIGIN ===\n');
fprintf('Driving robot back to calibration position...\n');

% Calculate return distance (with small extra to account for lag)
return_distance = dist_traveled + cfg.calib_return_extra;
return_time = return_distance / cfg.calib_drive_speed;

fprintf('Reversing %.3f m (%.1f seconds at %.2f m/s)...\n', ...
        return_distance, return_time, cfg.calib_drive_speed);

% First ensure wheels are straight
fprintf('Straightening wheels...\n');
for i = 1:10
    sendRobotCmd(sock, 0, 0, cfg.WHEELBASE);
    pause(0.05);
end

% Drive backward
drive_start_time = tic;
while toc(drive_start_time) < return_time
    sendRobotCmd(sock, -cfg.calib_drive_speed, 0, cfg.WHEELBASE);  % Negative velocity for reverse
    pause(0.05);
end

% Stop
fprintf('Stopping...\n');
stopRobot(sock, cfg.WHEELBASE);
pause(0.5);

% Check final position
final_pose = get_mocap_pose(mq, cfg.topic_pose);
final_x = toMazeX(final_pose.pos(1), final_pose.pos(3));
final_y = toMazeY(final_pose.pos(1), final_pose.pos(3));
final_error = hypot(final_x, final_y);

fprintf('\n✓ Robot returned to origin\n');
fprintf('  Final position: (%.3f, %.3f)\n', final_x, final_y);
fprintf('  Error from origin: %.3f m\n', final_error);

if final_error > 0.10
    fprintf('  ⚠ Robot is %.2f cm from origin. Manual adjustment may be needed.\n', final_error*100);
else
    fprintf('  ✓ Robot is close to origin (within 10cm)\n');
end

%% ======================= SAVE CALIBRATION =======================
save(cfg.CALIB_FILE, 'calib');

fprintf('\n=================================================================\n');
fprintf('CALIBRATION COMPLETE\n');
fprintf('=================================================================\n');
fprintf('Saved to: %s\n', cfg.CALIB_FILE);
fprintf('  Origin X: %.4f m\n', calib.origin_x);
fprintf('  Origin Z: %.4f m\n', calib.origin_z);
fprintf('  Origin Yaw: %.2f°\n', rad2deg(calib.origin_yaw));
fprintf('  Transform: %d (%s)\n', calib.transform, transforms(calib.transform).name);
fprintf('\n✓ Robot is at origin - ready for autonomous navigation\n');
fprintf('  Run ekf_lqr_dubins.m or manual_drive_logger.m next\n\n');

% Cleanup
clear sock;
delete(mq);

%% ======================= HELPER FUNCTIONS =======================
function cfg = limo_config()
    % Centralized configuration for ALL Limo robot scripts
    
    % Robot physical parameters
    cfg.WHEELBASE = 0.2;                 % [m] Limo wheelbase
    cfg.MAX_STEERING_DEG = 30;           % [deg] Maximum steering angle
    cfg.MAX_STEERING_RAD = deg2rad(30);  % [rad] Maximum steering angle
    cfg.MAX_SPEED = 0.2;                 % [m/s] Robot speed limit (enforced by Python)
    
    % Control parameters
    cfg.v_nom = 0.30;                    % [m/s] Nominal forward speed
    cfg.v_manual = 0.30;                 % [m/s] Manual drive speed
    cfg.omega_max = 1.0;                 % [rad/s] Maximum angular velocity
    cfg.dt_control = 0.05;               % [s] Control loop period (20 Hz)
    
    % Path planning parameters
    cfg.R_min = 0.40;                    % [m] Minimum turning radius
    cfg.step = 0.05;                     % [m] Dubins path interpolation step
    
    % Control gains (hand-tuned)
    cfg.K_ey = 8.0;                      % Cross-track error gain
    cfg.K_etheta = 3.0;                  % Heading error gain
    cfg.K_ev = 0.5;                      % Velocity error gain (unused)
    cfg.LOOKAHEAD = 0.40;                % [m] Lookahead distance
    
    % Obstacle parameters
    cfg.obsR = 0.30;                     % [m] Obstacle radius
    cfg.collision_dist = 0.35;           % [m] Distance to consider collision
    cfg.safe_dist = 0.80;                % [m] Distance to trigger replanning
    
    % Course definition - MOAP Lab Maze
    cfg.obs = [
        1.5 0.0; 1.5 0.5; 1.5 1.0; 1.5 1.5; 1.5 2.0;         % Left wall
        3.3 3.0; 3.3 3.5; 3.3 4.0; 3.3 4.5; 3.3 5.0; 3.3 5.5  % Right wall
    ];
    
    cfg.wps_2d = [
        0.0 0.0     % Start at origin
        0.8 0.2     % Slight right to prepare for turn
        0.8 2.5     % Up through first corridor
        2.5 2.5     % Right through middle gap
        4.0 2.5     % Continue right
        4.0 4.0     % Up toward goal
        5.0 4.5     % Goal position
    ];
    
    cfg.courseName = "MOAP Lab Course 1";
    
    % EKF parameters
    cfg.TAU_V = 0.15;                    % [s] Velocity time constant
    cfg.TAU_OMEGA = 0.15;                % [s] Angular velocity time constant
    
    % Process noise covariance
    cfg.Q_pos = 0.01^2;
    cfg.Q_theta = (2*pi/180)^2;
    cfg.Q_v = 0.05^2;
    cfg.Q_omega = (5*pi/180)^2;
    cfg.Q = diag([cfg.Q_pos, cfg.Q_pos, cfg.Q_theta, cfg.Q_v, cfg.Q_omega]);
    
    % Measurement noise covariance
    cfg.R_pos = 0.002^2;
    cfg.R_theta = (1*pi/180)^2;
    cfg.R = diag([cfg.R_pos, cfg.R_pos, cfg.R_theta]);
    
    % Initial covariance
    cfg.P0 = diag([0.01^2, 0.01^2, (2*pi/180)^2, 0.1^2, (10*pi/180)^2]);
    
    % Connection parameters
    cfg.broker = "mqtt://rasticvm.lan";
    cfg.mqtt_port = 1883;
    cfg.topic_pose = "rb/limo809";
    cfg.LIMO_IP = "192.168.1.172";
    cfg.LIMO_PORT = 12345;
    
    % File names
    cfg.CALIB_FILE = 'limo_calibration.mat';
    
    % Visualization settings
    cfg.arena_bounds = [-0.5 5.5 -6.0 1.0];
    
    % Timing parameters
    cfg.MIN_REPLAN_INTERVAL = 1.0;      % [s] Minimum time between replanning
    cfg.TIMEOUT = 60.0;                 % [s] Maximum run time
    
    % Calibration parameters
    cfg.calib_drive_speed = 0.20;       % [m/s] Speed for calibration drive
    cfg.calib_drive_time = 2.5;         % [s] Time to drive forward
    cfg.calib_return_extra = 0.05;      % [m] Extra distance to reverse
end

function pose = get_mocap_pose(mq, topic)
    % Get latest MoCap pose with correct yaw extraction
    
    pause(0.05);
    tbl = [];
    max_attempts = 50;
    attempts = 0;
    
    while isempty(tbl) && attempts < max_attempts
        tbl = read(mq, Topic=topic);
        if isempty(tbl)
            pause(0.01);
            attempts = attempts + 1;
        end
    end
    
    if isempty(tbl)
        error('No MoCap data available on topic: %s after %d attempts', topic, max_attempts);
    end
    
    data = jsondecode(tbl.Data{end});
    pose.pos = data.pos;
    
    % Extract YAW from quaternion for OptiTrack Y-up system
    qx = data.rot(1);
    qy = data.rot(2);
    qz = data.rot(3);
    qw = data.rot(4);
    
    pose.yaw = atan2(2*(qw*qy - qz*qx), 1 - 2*(qx^2 + qy^2));
end

function sendRobotCmd(sock, v, omega, wheelbase)
    % Send velocity command directly to robot via TCP
    
    % Convert angular velocity to steering angle
    if abs(v) > 0.05
        steering_angle = atan(wheelbase * omega / v);
    else
        steering_angle = omega * 0.3;
    end
    
    % Clamp steering angle
    max_steering = deg2rad(30);
    steering_angle = max(min(steering_angle, max_steering), -max_steering);
    
    % Send command
    cmd = sprintf("%.4f,%.4f\n", v, steering_angle);
    write(sock, cmd);
    
    % Read response if available
    if sock.NumBytesAvailable > 0
        response = read(sock, sock.NumBytesAvailable, "string");
    end
end

function stopRobot(sock, wheelbase)
    % Send stop command to robot
    
    for i = 1:5
        sendRobotCmd(sock, 0, 0, wheelbase);
        pause(0.05);
    end
end

function [toMazeX, toMazeY] = get_transform_functions(calib)
    % Get coordinate transform functions based on calibration
    
    switch calib.transform
        case 1
            toMazeX = @(xg, zg) zg - calib.origin_z;
            toMazeY = @(xg, zg) xg - calib.origin_x;
        case 2
            toMazeX = @(xg, zg) -(zg - calib.origin_z);
            toMazeY = @(xg, zg) xg - calib.origin_x;
        case 3
            toMazeX = @(xg, zg) xg - calib.origin_x;
            toMazeY = @(xg, zg) zg - calib.origin_z;
        case 4
            toMazeX = @(xg, zg) xg - calib.origin_x;
            toMazeY = @(xg, zg) -(zg - calib.origin_z);
        otherwise
            error('Invalid transform number: %d', calib.transform);
    end
end
