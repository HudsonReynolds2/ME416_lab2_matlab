clear; clc; close all;

%% ======================= MANUAL DRIVE WITH LOGGING =======================
% Drive the robot manually through the MOAP lab maze with full logging
% Uses WASD controls and logs everything for analysis
% Path generated automatically using A*
%
% CONTROLS: 
%   W/S = Forward/Backward
%   A/D = Turn left/right  
%   SPACE = Stop
%   Q = Quit and save

fprintf('=================================================================\n');
fprintf('MANUAL DRIVE LOGGER\n');
fprintf('=================================================================\n\n');

% Add utilities
addpath(fileparts(mfilename('fullpath')));

% Load configuration
cfg = limo_config();

%% ======================= LOAD CALIBRATION =======================
if ~exist(cfg.CALIB_FILE, 'file')
    error(['Calibration file not found!\n' ...
           'Run calibrate_limo.m first with robot at origin']);
end
load(cfg.CALIB_FILE, 'calib');
fprintf('✓ Calibration loaded (Transform %d)\n', calib.transform);

% Get transform functions
[toMazeX, toMazeY] = get_transform_functions(calib);

%% ======================= GENERATE PATH WITH A* =======================
fprintf('\n=== PATH PLANNING ===\n');

% Generate waypoints using A*
waypoints = plan_astar_path(cfg.start_pos, cfg.goal_pos, cfg.obs, cfg.obsR, cfg);

% Build smooth Dubins path through waypoints
fprintf('\nBuilding Dubins path through waypoints...\n');
[ref, ~] = build_dubins_path(waypoints, cfg.R_min, cfg.step, ...
                             cfg.v_nom, cfg.obs, cfg.obsR);
fprintf('✓ Path: %d points, %.2f m\n', length(ref.x), ref.s(end));

%% ======================= SETUP CONNECTIONS =======================
fprintf('\nSetting up connections...\n');

% MQTT for pose data
mq = mqttclient(cfg.broker, Port=cfg.mqtt_port, ClientID="manualDriver");
if ~mq.Connected
    error('Could not connect to MQTT broker');
end
subscribe(mq, cfg.topic_pose);
fprintf('✓ MQTT connected for pose data\n');

% TCP socket for robot control
sock = tcpclient(cfg.LIMO_IP, cfg.LIMO_PORT);
configureCallback(sock, "off");
fprintf('✓ TCP connected to robot\n\n');

fprintf('=== CONTROLS ===\n');
fprintf('  W/S     = Forward/Backward\n');
fprintf('  A/D     = Turn left/right\n');
fprintf('  SPACE   = Stop\n');
fprintf('  Q       = Quit and save\n\n');
fprintf('Press ENTER to start...\n');
input('');

%% ======================= SETUP VISUALIZATION =======================
setup_figure_fullscreen();

% Main plot
subplot(1,2,1);
hold on; grid on; axis equal;
xlim([cfg.arena_bounds(1) cfg.arena_bounds(2)]);
ylim([cfg.arena_bounds(3) cfg.arena_bounds(4)]);
title(sprintf('%s - Manual Drive', cfg.courseName), 'FontSize', 16, 'FontWeight', 'bold');
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

legend({'Arena', 'Obstacles', '', 'Waypoints', 'Path', 'Start', 'Trail', 'Robot'}, ...
       'Location', 'northwest', 'FontSize', 10);

% Info panel
subplot(1,2,2);
axis off;
h_info = text(0.05, 0.95, '', 'FontSize', 11, 'FontName', 'Courier', ...
              'VerticalAlignment', 'top', 'Interpreter', 'none');

%% ======================= KEYBOARD SETUP =======================
set(gcf, 'WindowKeyPressFcn', @keyPress);
set(gcf, 'WindowKeyReleaseFcn', @keyRelease);

global keys running
keys = struct('w', 0, 's', 0, 'a', 0, 'd', 0);
running = true;

%% ======================= DATA LOGGING =======================
log = struct();
log.time = [];
log.x_pos = []; log.y_pos = []; log.theta_pos = [];
log.x_ref = []; log.y_ref = []; log.theta_ref = [];
log.e_y = []; log.e_theta = [];
log.v_cmd = []; log.omega_cmd = [];
log.s_current = [];
log.keys = {};

trail_x = [];
trail_y = [];

%% ======================= MAIN LOOP =======================
fprintf('\n=== DRIVING - Use WASD keys ===\n');

t_start = tic;
iter = 0;

try
    while running
        iter = iter + 1;
        t = toc(t_start);
        
        %% --- GET COMMANDS FROM KEYS ---
        v_cmd = cfg.v_manual * (keys.w - keys.s);
        omega_cmd = 0.5 * (keys.a - keys.d);
        
        %% --- SEND COMMAND ---
        sendRobotCmd(sock, v_cmd, omega_cmd, cfg.WHEELBASE);
        
        %% --- GET POSE ---
        pose = get_mocap_pose(mq, cfg.topic_pose);
        [x_pos, y_pos, theta_pos] = transform_pose(pose, calib);
        
        %% --- FIND REFERENCE ---
        dists = sqrt((ref.x - x_pos).^2 + (ref.y - y_pos).^2);
        [~, idx_closest] = min(dists);
        s_current = ref.s(idx_closest);
        
        % Look ahead for reference
        s_target = min(s_current + cfg.LOOKAHEAD, ref.s(end));
        idx_ref = find(ref.s >= s_target, 1);
        if isempty(idx_ref)
            idx_ref = length(ref.s);
        end
        
        x_ref = ref.x(idx_ref);
        y_ref = ref.y(idx_ref);
        theta_ref = ref.theta(idx_ref);
        
        %% --- COMPUTE TRACKING ERRORS ---
        dx = x_pos - x_ref;
        dy = y_pos - y_ref;
        
        e_y = -dx * sin(theta_ref) + dy * cos(theta_ref);
        e_theta = wrapToPi(theta_pos - theta_ref);
        
        % What auto control would compute
        omega_auto = -(cfg.K_ey * e_y + cfg.K_etheta * e_theta);
        
        %% --- LOG DATA ---
        log.time(end+1) = t;
        log.x_pos(end+1) = x_pos;
        log.y_pos(end+1) = y_pos;
        log.theta_pos(end+1) = theta_pos;
        log.x_ref(end+1) = x_ref;
        log.y_ref(end+1) = y_ref;
        log.theta_ref(end+1) = theta_ref;
        log.e_y(end+1) = e_y;
        log.e_theta(end+1) = e_theta;
        log.v_cmd(end+1) = v_cmd;
        log.omega_cmd(end+1) = omega_cmd;
        log.s_current(end+1) = s_current;
        log.keys{end+1} = keys;
        
        %% --- UPDATE VISUALIZATION ---
        trail_x(end+1) = x_pos;
        trail_y(end+1) = y_pos;
        
        set(h_trail, 'XData', trail_x, 'YData', trail_y);
        set(h_robot, 'XData', x_pos, 'YData', y_pos);
        set(h_arrow, 'XData', x_pos, 'YData', y_pos, ...
            'UData', 0.3*cos(theta_pos), 'VData', 0.3*sin(theta_pos));
        set(h_target, 'XData', x_ref, 'YData', y_ref);
        
        % Update info
        info_text = sprintf('Time: %.1f s\n\n', t);
        info_text = [info_text sprintf('POSITION:\n')];
        info_text = [info_text sprintf('  Robot: (%.3f, %.3f) @ %.1f°\n', x_pos, y_pos, rad2deg(theta_pos))];
        info_text = [info_text sprintf('  Ref:   (%.3f, %.3f) @ %.1f°\n\n', x_ref, y_ref, rad2deg(theta_ref))];
        info_text = [info_text sprintf('PROGRESS: %.1f / %.1f m (%.0f%%)\n\n', ...
                            s_current, ref.s(end), 100*s_current/ref.s(end))];
        info_text = [info_text sprintf('TRACKING ERRORS:\n')];
        info_text = [info_text sprintf('  Lateral: %.3f m\n', e_y)];
        info_text = [info_text sprintf('  Heading: %.1f°\n\n', rad2deg(e_theta))];
        info_text = [info_text sprintf('MANUAL CONTROL:\n')];
        info_text = [info_text sprintf('  v = %.2f m/s\n', v_cmd)];
        info_text = [info_text sprintf('  ω = %.1f°/s\n\n', rad2deg(omega_cmd))];
        info_text = [info_text sprintf('AUTO WOULD USE:\n')];
        info_text = [info_text sprintf('  ω = %.1f°/s\n\n', rad2deg(omega_auto))];
        
        % Show active keys
        active_keys = '';
        if keys.w, active_keys = [active_keys 'W ']; end
        if keys.s, active_keys = [active_keys 'S ']; end
        if keys.a, active_keys = [active_keys 'A ']; end
        if keys.d, active_keys = [active_keys 'D ']; end
        if isempty(active_keys), active_keys = 'None'; end
        info_text = [info_text sprintf('KEYS: %s', active_keys)];
        
        set(h_info, 'String', info_text);
        drawnow limitrate;
        
        % Status print
        if mod(iter, 20) == 0
            fprintf('\r[%.1fs] Pos:(%.2f,%.2f) | Progress:%.0f%% | Keys:%s    ', ...
                    t, x_pos, y_pos, 100*s_current/ref.s(end), active_keys);
        end
        
        % Check goal
        dist_to_goal = hypot(x_pos - ref.x(end), y_pos - ref.y(end));
        if dist_to_goal < 0.2
            fprintf('\n✓ GOAL REACHED!\n');
            running = false;
        end
        
        pause(cfg.dt_control);
    end
    
catch ME
    if ~strcmp(ME.identifier, 'MATLAB:interrupt')
        fprintf('\nError: %s\n', ME.message);
    end
    fprintf('\n*** STOPPED ***\n');
end

%% ======================= SAVE AND CLEANUP =======================
fprintf('\n\n=== STOPPING ===\n');

% Stop robot
stopRobot(sock, cfg.WHEELBASE);

% Save logs
log.cfg = cfg;
log.calib = calib;
log.ref = ref;
log.waypoints = waypoints;

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('manual_drive_%s.mat', timestamp);
save(filename, 'log');
fprintf('✓ Logs saved to: %s\n', filename);

% Report
fprintf('\n=== DRIVE COMPLETE ===\n');
fprintf('Duration: %.1f s\n', log.time(end));
fprintf('Samples: %d\n', length(log.time));
fprintf('Distance on path: %.2f m\n', log.s_current(end));

% Analyze tracking
mean_lateral_error = mean(abs(log.e_y));
mean_heading_error = mean(abs(log.e_theta));
fprintf('\nTracking performance:\n');
fprintf('  Mean lateral error: %.3f m\n', mean_lateral_error);
fprintf('  Mean heading error: %.1f°\n', rad2deg(mean_heading_error));

% Cleanup
clear sock;
delete(mq);
close(gcf);

fprintf('\n=================================================================\n\n');

%% ======================= CALLBACK FUNCTIONS =======================
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
