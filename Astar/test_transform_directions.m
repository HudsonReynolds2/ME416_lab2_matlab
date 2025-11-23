clear; clc; close all;
% Running this (sitting below the maze axis) clearly shows T4 yaw+ is
% correct.
%% TEST TRANSFORM DIRECTIONS
% Shows 8 possible transform combinations
% Drive robot in a circle and see which visualization matches reality

fprintf('=================================================================\n');
fprintf('TRANSFORM DIRECTION TEST\n');
fprintf('=================================================================\n\n');

cfg = limo_config();

%% Load calibration
if ~exist(cfg.CALIB_FILE, 'file')
    error('Calibration file not found!');
end
load(cfg.CALIB_FILE, 'calib');
fprintf('Loaded calibration: Transform %d\n', calib.transform);

%% Setup connections
fprintf('Connecting...\n');
mq = mqttclient(cfg.broker, Port=cfg.mqtt_port, ClientID="transformTest");
if ~mq.Connected
    error('Could not connect to MQTT');
end
subscribe(mq, cfg.topic_pose);

sock = tcpclient(cfg.LIMO_IP, cfg.LIMO_PORT);
configureCallback(sock, "off");
fprintf('✓ Connected\n\n');

%% Get current pose
fprintf('Reading current pose...\n');
pose = get_mocap_pose(mq, cfg.topic_pose);
fprintf('MoCap position: (%.3f, %.3f, %.3f)\n', pose.pos(1), pose.pos(2), pose.pos(3));
fprintf('MoCap yaw: %.1f°\n\n', rad2deg(pose.yaw));

%% Setup figure with 8 subplots (4 transforms × 2 yaw signs)
figure('Units', 'normalized', 'Position', [0 0 1 1]);

% Store handles for each subplot
h_robot = zeros(8, 1);
h_arrow = zeros(8, 1);
h_trail = zeros(8, 1);
trail_x = cell(8, 1);
trail_y = cell(8, 1);
for i = 1:8
    trail_x{i} = [];
    trail_y{i} = [];
end
transform_labels = cell(8, 1);

subplot_idx = 1;
for trans = 1:4
    for yaw_sign = [-1, 1]
        subplot(2, 4, subplot_idx);
        hold on; grid on; axis equal;
        xlim([-5 5]); ylim([-5 5]);
        
        if yaw_sign == 1
            transform_labels{subplot_idx} = sprintf('T%d Yaw+', trans);
        else
            transform_labels{subplot_idx} = sprintf('T%d Yaw-', trans);
        end
        title(transform_labels{subplot_idx}, 'FontSize', 14, 'FontWeight', 'bold');
        xlabel('X [m]'); ylabel('Y [m]');
        
        % Draw origin
        plot(0, 0, 'go', 'MarkerSize', 20, 'MarkerFaceColor', 'g', 'LineWidth', 2);
        
        % Robot, arrow, and trail placeholders
        h_trail(subplot_idx) = plot(NaN, NaN, 'b-', 'LineWidth', 2);
        h_robot(subplot_idx) = plot(NaN, NaN, 'bo', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
        h_arrow(subplot_idx) = quiver(0, 0, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
        
        subplot_idx = subplot_idx + 1;
    end
end

%% Setup keyboard
set(gcf, 'WindowKeyPressFcn', @keyPress);
set(gcf, 'WindowKeyReleaseFcn', @keyRelease);

global keys running
keys = struct('w', 0, 's', 0, 'a', 0, 'd', 0);
running = true;

fprintf('=== CONTROLS ===\n');
fprintf('  W/S     = Forward/Backward\n');
fprintf('  A/D     = Turn left/right\n');
fprintf('  SPACE   = Stop\n');
fprintf('  Q       = Quit\n\n');
fprintf('Press ENTER to start...\n');
input('');

%% Main loop - continuously update all 8 visualizations
fprintf('\n=== SHOWING ALL 8 TRANSFORM OPTIONS ===\n');
fprintf('Drive the robot and see which visualization matches reality!\n');
fprintf('Which subplot (1-8) shows correct motion?\n\n');

try
    while running
        %% Get commands from keys
        v_cmd = cfg.v_manual * (keys.w - keys.s);
        omega_cmd = 0.5 * (keys.a - keys.d);
        
        %% Send command
        sendRobotCmd(sock, v_cmd, omega_cmd, cfg.WHEELBASE);
        
        %% Get pose
        pose = get_mocap_pose(mq, cfg.topic_pose);
        
        %% Update all 8 subplots
        subplot_idx = 1;
        for trans = 1:4
            for yaw_sign = [-1, 1]
                % Apply transform
                [x_maze, y_maze, theta_maze] = apply_transform(pose, calib, trans, yaw_sign);
                
                % Add to trail
                trail_x{subplot_idx}(end+1) = x_maze;
                trail_y{subplot_idx}(end+1) = y_maze;
                
                % Update plot
                set(h_trail(subplot_idx), 'XData', trail_x{subplot_idx}, 'YData', trail_y{subplot_idx});
                set(h_robot(subplot_idx), 'XData', x_maze, 'YData', y_maze);
                arrow_len = 0.3;
                set(h_arrow(subplot_idx), 'XData', x_maze, 'YData', y_maze, ...
                    'UData', arrow_len*cos(theta_maze), 'VData', arrow_len*sin(theta_maze));
                
                subplot_idx = subplot_idx + 1;
            end
        end
        
        drawnow limitrate;
        pause(cfg.dt_control);
    end
catch ME
    if ~strcmp(ME.identifier, 'MATLAB:interrupt')
        fprintf('\nError: %s\n', ME.message);
    end
    fprintf('\n*** STOPPED ***\n');
end

%% Cleanup
stopRobot(sock, cfg.WHEELBASE);
clear sock;
delete(mq);

fprintf('\n=================================================================\n');
fprintf('Which subplot (1-8) showed correct motion?\n');
fprintf('Update your code to use that transform and yaw sign.\n');
fprintf('=================================================================\n\n');

%% Callback functions
function keyPress(~, evt)
    global keys running
    switch lower(evt.Key)
        case 'w', keys.w = 1;
        case 's', keys.s = 1;
        case 'a', keys.a = 1;
        case 'd', keys.d = 1;
        case 'space'
            keys.w = 0; keys.s = 0; keys.a = 0; keys.d = 0;
        case 'q'
            running = false;
    end
end

function keyRelease(~, evt)
    global keys
    switch lower(evt.Key)
        case 'w', keys.w = 0;
        case 's', keys.s = 0;
        case 'a', keys.a = 0;
        case 'd', keys.d = 0;
    end
end

%% Helper functions
function [x_maze, y_maze, theta_maze] = apply_transform(pose, calib, trans, yaw_sign)
    % Apply specified transform
    switch trans
        case 1
            x_maze = pose.pos(3) - calib.origin_z;
            y_maze = pose.pos(1) - calib.origin_x;
        case 2
            x_maze = -(pose.pos(3) - calib.origin_z);
            y_maze = pose.pos(1) - calib.origin_x;
        case 3
            x_maze = pose.pos(1) - calib.origin_x;
            y_maze = pose.pos(3) - calib.origin_z;
        case 4
            x_maze = pose.pos(1) - calib.origin_x;
            y_maze = -(pose.pos(3) - calib.origin_z);
    end
    
    theta_maze = yaw_sign * (pose.yaw - calib.origin_yaw);
    theta_maze = wrapToPi(theta_maze);
end

function pose = get_mocap_pose(mq, topic)
    pause(0.05);
    tbl = read(mq, Topic=topic);
    if isempty(tbl)
        error('No MoCap data');
    end
    
    data = jsondecode(tbl.Data{end});
    pose.pos = data.pos;
    
    qx = data.rot(1);
    qy = data.rot(2);
    qz = data.rot(3);
    qw = data.rot(4);
    
    pose.yaw = atan2(2*(qw*qy - qz*qx), 1 - 2*(qx^2 + qy^2));
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
end

function stopRobot(sock, wheelbase)
    for i = 1:5
        sendRobotCmd(sock, 0, 0, wheelbase);
        pause(0.05);
    end
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end
