clear; clc; close all;

%% ======================= AUTONOMOUS MAZE NAVIGATION =======================
% EKF + Lateral Error Control for autonomous navigation
% Uses A* path planning with Dubins smoothing
% Transform 4 with positive yaw (verified correct)

fprintf('=================================================================\n');
fprintf('AUTONOMOUS MAZE NAVIGATION\n');
fprintf('=================================================================\n\n');

% Load configuration
cfg = limo_config();

%% ======================= SELECT MAZE =======================
fprintf('Available mazes:\n');
for i = 1:length(cfg.mazes)
    fprintf('  %d. %s\n', i, cfg.mazes(i).name);
end
maze_choice = input(sprintf('Select maze (1-%d): ', length(cfg.mazes)));

if maze_choice < 1 || maze_choice > length(cfg.mazes)
    error('Invalid maze selection!');
end

% Set selected maze
cfg.maze_select = maze_choice;
cfg.obs = cfg.mazes(maze_choice).obs;
cfg.start_pos = cfg.mazes(maze_choice).start_pos;
cfg.goal_pos = cfg.mazes(maze_choice).goal_pos;
cfg.arena_bounds = cfg.mazes(maze_choice).arena_bounds;
cfg.courseName = cfg.mazes(maze_choice).name;

fprintf('\n✓ Selected: %s\n', cfg.courseName);

%% ======================= LOAD CALIBRATION =======================
if ~exist(cfg.CALIB_FILE, 'file')
    error(['Calibration file not found!\n' ...
           'Run calibrate_limo.m first with robot at origin']);
end
load(cfg.CALIB_FILE, 'calib');
fprintf('✓ Calibration loaded (Transform %d in file)\n', calib.transform);

% Force Transform 4 (verified correct)
calib.transform = 4;
fprintf('✓ Using Transform 4 with positive yaw (verified)\n');

%% ======================= GENERATE PATH WITH A* =======================
fprintf('\n=== PATH PLANNING ===\n');

% Generate waypoints using A*
waypoints = plan_astar_path(cfg.start_pos, cfg.goal_pos, cfg.obs, cfg.obsR, cfg);
fprintf('✓ A* waypoints: %d\n', size(waypoints, 1));

% Build smooth Dubins path through waypoints
fprintf('Building Dubins path through waypoints...\n');
[ref, ~] = build_dubins_path(waypoints, cfg.R_min, cfg.step, ...
                             cfg.v_nom, cfg.obs, cfg.obsR);
fprintf('✓ Path complete: %d points, %.2f m\n', length(ref.x), ref.s(end));

%% ======================= SETUP CONNECTIONS =======================
fprintf('\n=== CONNECTING ===\n');

% MQTT for pose data
mq = mqttclient(cfg.broker, Port=cfg.mqtt_port, ClientID="autoNav");
if ~mq.Connected
    error('Could not connect to MQTT broker');
end
subscribe(mq, cfg.topic_pose);
fprintf('✓ MQTT connected\n');

% TCP socket for robot control
sock = tcpclient(cfg.LIMO_IP, cfg.LIMO_PORT);
configureCallback(sock, "off");
fprintf('✓ TCP connected to robot\n');

%% ======================= INITIALIZE EKF =======================
fprintf('\n=== INITIALIZING EKF ===\n');

% Get initial pose
pose = get_mocap_pose(mq, cfg.topic_pose);
[x_init, y_init, theta_init] = transform_pose(pose, calib);

% Initialize state: [x, y, theta, v, omega]'
x_ekf = [x_init; y_init; theta_init; 0; 0];
P_ekf = cfg.P0;

fprintf('✓ Initial pose: (%.2f, %.2f) @ %.0f°\n', ...
        x_init, y_init, rad2deg(theta_init));

% Check if robot is near start
start_error = hypot(x_init - ref.x(1), y_init - ref.y(1));
if start_error > 0.3
    fprintf('\n⚠ WARNING: Robot is %.2f m from path start!\n', start_error);
    fprintf('  Expected: (%.2f, %.2f)\n', ref.x(1), ref.y(1));
    fprintf('  Actual:   (%.2f, %.2f)\n', x_init, y_init);
    response = input('Continue anyway? (y/n): ', 's');
    if ~strcmpi(response, 'y')
        stopRobot(sock, cfg.WHEELBASE);
        clear sock;
        delete(mq);
        return;
    end
end

%% ======================= SETUP VISUALIZATION =======================
fprintf('\nSetting up visualization...\n');
setup_figure_fullscreen();

% Main plot
subplot(1,2,1);
hold on; grid on; axis equal;
xlim([cfg.arena_bounds(1) cfg.arena_bounds(2)]);
ylim([cfg.arena_bounds(3) cfg.arena_bounds(4)]);
title(sprintf('%s - Autonomous Navigation', cfg.courseName), ...
      'FontSize', 16, 'FontWeight', 'bold');
xlabel('X [m]'); ylabel('Y [m]');

% Static elements
plot_arena(cfg.arena_bounds);
plot_obstacles(cfg.obs, cfg.obsR);
plot_path(ref, waypoints);

% Dynamic elements
h_trail = plot(NaN, NaN, 'b-', 'LineWidth', 3);
h_robot = plot(NaN, NaN, 'bo', 'MarkerSize', 20, 'MarkerFaceColor', 'b');
h_arrow = quiver(0, 0, 0, 0, 'b', 'LineWidth', 4, 'MaxHeadSize', 0.5);
h_target = plot(NaN, NaN, 'rs', 'MarkerSize', 20, 'LineWidth', 3);

legend({'Arena', 'Obstacles', '', 'Waypoints', 'Path', 'Start', ...
        'Trail', 'Robot', 'Target'}, 'Location', 'northwest');

% Info panel
subplot(1,2,2);
axis off;
h_info = text(0.05, 0.95, '', 'FontSize', 11, 'FontName', 'Courier', ...
              'VerticalAlignment', 'top', 'Interpreter', 'none');

fprintf('✓ Visualization ready\n');

%% ======================= MAIN CONTROL LOOP =======================
fprintf('\n=== STARTING AUTONOMOUS NAVIGATION ===\n');
fprintf('Press Ctrl+C to stop\n\n');

% Initialize logging
log = struct();
log.time = [];
log.x_meas = []; log.y_meas = []; log.theta_meas = [];
log.x_ekf = []; log.y_ekf = []; log.theta_ekf = [];
log.v_ekf = []; log.omega_ekf = [];
log.x_ref = []; log.y_ref = []; log.theta_ref = [];
log.e_y = []; log.e_theta = [];
log.v_cmd = []; log.omega_cmd = [];
log.s_current = [];

trail_x = [];
trail_y = [];

t_start = tic;
iter = 0;
goal_reached = false;
v_cmd = 0;
omega_cmd = 0;

try
    while ~goal_reached && toc(t_start) < cfg.TIMEOUT
        iter = iter + 1;
        t = toc(t_start);
        
        %% --- EKF PREDICTION ---
        [x_ekf, P_ekf] = ekf_predict(x_ekf, P_ekf, v_cmd, omega_cmd, ...
                                      cfg.dt_control, cfg.Q, cfg.TAU_V, cfg.TAU_OMEGA);
        
        %% --- GET MEASUREMENT ---
        pose = get_mocap_pose(mq, cfg.topic_pose);
        [x_meas, y_meas, theta_meas] = transform_pose(pose, calib);
        
        %% --- EKF UPDATE ---
        z = [x_meas; y_meas; theta_meas];
        [x_ekf, P_ekf] = ekf_update(x_ekf, P_ekf, z, cfg.R);
        
        %% --- EXTRACT ESTIMATES ---
        x_est = x_ekf(1);
        y_est = x_ekf(2);
        theta_est = x_ekf(3);
        v_est = x_ekf(4);
        omega_est = x_ekf(5);
        
        %% --- COMPUTE CONTROL ---
        [v_cmd, omega_cmd, e_y, e_theta, idx_ref] = compute_control(...
            x_est, y_est, theta_est, ref, ...
            cfg.K_ey, cfg.K_etheta, cfg.LOOKAHEAD, cfg.v_nom);
        
        % Get current progress
        dists = sqrt((ref.x - x_est).^2 + (ref.y - y_est).^2);
        [~, idx_closest] = min(dists);
        s_current = ref.s(idx_closest);
        
        % Reference point
        x_ref = ref.x(idx_ref);
        y_ref = ref.y(idx_ref);
        theta_ref = ref.theta(idx_ref);
        
        %% --- SEND COMMAND ---
        sendRobotCmd(sock, v_cmd, omega_cmd, cfg.WHEELBASE);
        
        %% --- CHECK GOAL ---
        dist_to_goal = hypot(x_est - ref.x(end), y_est - ref.y(end));
        if dist_to_goal < 0.2
            goal_reached = true;
        end
        
        %% --- LOG DATA ---
        log.time(end+1) = t;
        log.x_meas(end+1) = x_meas;
        log.y_meas(end+1) = y_meas;
        log.theta_meas(end+1) = theta_meas;
        log.x_ekf(end+1) = x_est;
        log.y_ekf(end+1) = y_est;
        log.theta_ekf(end+1) = theta_est;
        log.v_ekf(end+1) = v_est;
        log.omega_ekf(end+1) = omega_est;
        log.x_ref(end+1) = x_ref;
        log.y_ref(end+1) = y_ref;
        log.theta_ref(end+1) = theta_ref;
        log.e_y(end+1) = e_y;
        log.e_theta(end+1) = e_theta;
        log.v_cmd(end+1) = v_cmd;
        log.omega_cmd(end+1) = omega_cmd;
        log.s_current(end+1) = s_current;
        
        %% --- UPDATE VISUALIZATION ---
        trail_x(end+1) = x_est;
        trail_y(end+1) = y_est;
        
        set(h_trail, 'XData', trail_x, 'YData', trail_y);
        set(h_robot, 'XData', x_est, 'YData', y_est);
        set(h_arrow, 'XData', x_est, 'YData', y_est, ...
            'UData', 0.3*cos(theta_est), 'VData', 0.3*sin(theta_est));
        set(h_target, 'XData', x_ref, 'YData', y_ref);
        
        % Update info text
        info_text = sprintf('Time: %.1f s\n\n', t);
        info_text = [info_text sprintf('MEASURED POSE:\n')];
        info_text = [info_text sprintf('  X:     %.3f m\n', x_meas)];
        info_text = [info_text sprintf('  Y:     %.3f m\n', y_meas)];
        info_text = [info_text sprintf('  Theta: %.1f°\n\n', rad2deg(theta_meas))];
        
        info_text = [info_text sprintf('EKF ESTIMATE:\n')];
        info_text = [info_text sprintf('  X:     %.3f m\n', x_est)];
        info_text = [info_text sprintf('  Y:     %.3f m\n', y_est)];
        info_text = [info_text sprintf('  Theta: %.1f°\n', rad2deg(theta_est))];
        info_text = [info_text sprintf('  v:     %.3f m/s\n', v_est)];
        info_text = [info_text sprintf('  omega: %.1f°/s\n\n', rad2deg(omega_est))];
        
        info_text = [info_text sprintf('PROGRESS: %.1f / %.1f m (%.0f%%)\n\n', ...
                            s_current, ref.s(end), 100*s_current/ref.s(end))];
        
        info_text = [info_text sprintf('TRACKING ERRORS:\n')];
        info_text = [info_text sprintf('  Lateral: %.3f m\n', e_y)];
        info_text = [info_text sprintf('  Heading: %.1f°\n\n', rad2deg(e_theta))];
        
        info_text = [info_text sprintf('CONTROL:\n')];
        info_text = [info_text sprintf('  v:     %.3f m/s\n', v_cmd)];
        info_text = [info_text sprintf('  omega: %.1f°/s\n\n', rad2deg(omega_cmd))];
        
        info_text = [info_text sprintf('DISTANCE TO GOAL: %.2f m', dist_to_goal)];
        
        set(h_info, 'String', info_text);
        drawnow limitrate;
        
        % Status print
        if mod(iter, 20) == 0
            fprintf('\r[%.1fs] Pos:(%.2f,%.2f) | Progress:%.0f%% | Errors: e_y=%.3f e_θ=%.1f°', ...
                    t, x_est, y_est, 100*s_current/ref.s(end), e_y, rad2deg(e_theta));
        end
        
        pause(cfg.dt_control);
    end
    
catch ME
    if ~strcmp(ME.identifier, 'MATLAB:interrupt')
        fprintf('\nError: %s\n', ME.message);
    end
    fprintf('\n*** STOPPED ***\n');
end

%% ======================= CLEANUP =======================
fprintf('\n\n=== STOPPING ===\n');

% Stop robot
stopRobot(sock, cfg.WHEELBASE);

if goal_reached
    fprintf('✓ GOAL REACHED!\n');
else
    fprintf('⚠ Did not reach goal (timeout or stopped)\n');
end

% Save logs
log.cfg = cfg;
log.calib = calib;
log.ref = ref;
log.waypoints = waypoints;

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('autonomous_nav_%s.mat', timestamp);
save(filename, 'log');
fprintf('✓ Logs saved to: %s\n', filename);

% Report performance
fprintf('\n=== PERFORMANCE ===\n');
fprintf('Duration: %.1f s\n', log.time(end));
fprintf('Distance traveled: %.2f m\n', log.s_current(end));
fprintf('Mean lateral error: %.3f m\n', mean(abs(log.e_y)));
fprintf('Mean heading error: %.1f°\n', rad2deg(mean(abs(log.e_theta))));
fprintf('Max lateral error: %.3f m\n', max(abs(log.e_y)));
fprintf('Max heading error: %.1f°\n', rad2deg(max(abs(log.e_theta))));

% Cleanup
clear sock;
delete(mq);

fprintf('\n=================================================================\n\n');

%% ======================= UTILITY FUNCTIONS =======================
function [toMazeX, toMazeY] = get_transform_functions(calib)
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

function pose = get_mocap_pose(mq, topic)
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
        error('No MoCap data available');
    end
    
    data = jsondecode(tbl.Data{end});
    pose.pos = data.pos;
    
    qx = data.rot(1);
    qy = data.rot(2);
    qz = data.rot(3);
    qw = data.rot(4);
    
    pose.yaw = atan2(2*(qw*qy - qz*qx), 1 - 2*(qx^2 + qy^2));
end

function [x_maze, y_maze, theta_maze] = transform_pose(pose, calib)
    [toMazeX, toMazeY] = get_transform_functions(calib);
    x_maze = toMazeX(pose.pos(1), pose.pos(3));
    y_maze = toMazeY(pose.pos(1), pose.pos(3));
    theta_maze = wrapToPi(pose.yaw - calib.origin_yaw);  % Positive yaw
end

function sendRobotCmd(sock, v, omega, wheelbase)
    if abs(v) > 0.05
        steering_angle = atan(wheelbase * omega / v);
    else
        steering_angle = omega * 0.3;
    end
    
    max_steering = deg2rad(30);
    steering_angle = max(min(steering_angle, max_steering), -max_steering);
    
    cmd = sprintf("%.4f,%.4f\n", v, steering_angle);
    write(sock, cmd);
    
    if sock.NumBytesAvailable > 0
        response = read(sock, sock.NumBytesAvailable, "string");
    end
end

function stopRobot(sock, wheelbase)
    for i = 1:5
        sendRobotCmd(sock, 0, 0, wheelbase);
        pause(0.05);
    end
end

function [x_pred, P_pred] = ekf_predict(x, P, v_cmd, omega_cmd, dt, Q, tau_v, tau_omega)
    x = x(:);
    
    % Extract states
    theta = x(3);
    v = x(4);
    omega = x(5);
    
    % Continuous-time dynamics
    f = [v * cos(theta);
         v * sin(theta);
         omega;
         (v_cmd - v) / tau_v;
         (omega_cmd - omega) / tau_omega];
    
    % Discrete-time prediction
    x_pred = x + f * dt;
    x_pred(3) = wrapToPi(x_pred(3));
    
    % Jacobian
    F = [0, 0, -v*sin(theta), cos(theta), 0;
         0, 0,  v*cos(theta), sin(theta), 0;
         0, 0,  0,            0,          1;
         0, 0,  0,           -1/tau_v,    0;
         0, 0,  0,            0,         -1/tau_omega];
    
    % Discrete-time state transition
    A = eye(5) + F * dt;
    
    % Predict covariance
    P_pred = A * P * A' + Q * dt;
end

function [x_upd, P_upd] = ekf_update(x, P, z, R)
    x = x(:);
    z = z(:);
    
    % Measurement model: z = [x; y; theta]
    H = [1, 0, 0, 0, 0;
         0, 1, 0, 0, 0;
         0, 0, 1, 0, 0];
    
    % Innovation
    y = z - H * x;
    y(3) = wrapToPi(y(3));
    
    % Innovation covariance
    S = H * P * H' + R;
    
    % Kalman gain
    K = P * H' / S;
    
    % Update state
    x_upd = x + K * y;
    x_upd(3) = wrapToPi(x_upd(3));
    
    % Update covariance (Joseph form)
    I_KH = eye(5) - K * H;
    P_upd = I_KH * P * I_KH' + K * R * K';
end

function [v_cmd, omega_cmd, e_y, e_theta, idx_ref] = compute_control(x_pos, y_pos, theta_pos, ref, K_ey, K_etheta, lookahead, v_nom)
    % Find closest point
    dists = sqrt((ref.x - x_pos).^2 + (ref.y - y_pos).^2);
    [~, idx_closest] = min(dists);
    
    % Look ahead
    s_current = ref.s(idx_closest);
    s_target = min(s_current + lookahead, ref.s(end));
    idx_ref = find(ref.s >= s_target, 1);
    if isempty(idx_ref)
        idx_ref = length(ref.s);
    end
    
    % Reference point
    x_ref = ref.x(idx_ref);
    y_ref = ref.y(idx_ref);
    theta_ref = ref.theta(idx_ref);
    
    % Compute errors
    dx = x_pos - x_ref;
    dy = y_pos - y_ref;
    
    % Lateral error (cross-track)
    e_y = -dx * sin(theta_ref) + dy * cos(theta_ref);
    
    % Heading error
    e_theta = wrapToPi(theta_pos - theta_ref);
    
    % Control law
    v_cmd = v_nom;
    omega_cmd = -(K_ey * e_y + K_etheta * e_theta);
    
    % Clamp
    omega_max = 1.0;
    omega_cmd = max(min(omega_cmd, omega_max), -omega_max);
end

function setup_figure_fullscreen()
    figure('Units', 'normalized', 'Position', [0 0 1 1]);
end

function plot_arena(bounds)
    plot([bounds(1) bounds(2) bounds(2) bounds(1) bounds(1)], ...
         [bounds(3) bounds(3) bounds(4) bounds(4) bounds(3)], ...
         'k', 'LineWidth', 3);
end

function plot_obstacles(obs, obsR)
    if isempty(obs)
        return;
    end
    
    scatter(obs(:,1), obs(:,2), 150, 'k', 'filled');
    for i = 1:size(obs, 1)
        viscircles([obs(i,1), obs(i,2)], obsR, ...
                   'Color', [0.8 0 0], 'LineStyle', '--', 'LineWidth', 2);
    end
end

function plot_path(ref, waypoints)
    plot(ref.x, ref.y, 'r--', 'LineWidth', 3);
    plot(waypoints(:,1), waypoints(:,2), 'mo', 'MarkerSize', 15, 'MarkerFaceColor', 'm');
    plot(ref.x(1), ref.y(1), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end
