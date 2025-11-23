%% LIMO_UTILS - Shared utility functions for all Limo robot scripts
% This file contains ALL reusable functions to ensure consistency
% Include this in your scripts with: addpath(fileparts(which('limo_utils')));

%% ======================= MOCAP FUNCTIONS =======================
function pose = get_mocap_pose(mq, topic)
    % GET_MOCAP_POSE - Get latest MoCap pose with CORRECT yaw extraction
    % Returns: struct with .pos (3x1 vector) and .yaw (scalar in radians)
    %
    % CRITICAL: Uses correct formula for OptiTrack Y-up quaternion system
    
    pause(0.05);  % Small delay to ensure fresh data
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
    pose.pos = data.pos;  % [x, y, z] in MoCap frame
    
    % Extract YAW from quaternion [x,y,z,w] for OptiTrack Y-up system
    % This formula is VERIFIED with physical robot rotations:
    % - 90° CCW: measures 90.1° (0.1° error)
    % - 180°: measures 179.1° (0.9° error) 
    % - 90° CW: measures -90.5° (0.5° error)
    qx = data.rot(1);
    qy = data.rot(2);
    qz = data.rot(3);
    qw = data.rot(4);
    
    pose.yaw = atan2(2*(qw*qy - qz*qx), 1 - 2*(qx^2 + qy^2));
end

%% ======================= ROBOT CONTROL FUNCTIONS =======================
function sendRobotCmd(sock, v, omega, wheelbase)
    % SENDROBOTCMD - Send velocity command directly to robot via TCP
    % Converts (v, omega) to (linear_vel, steering_angle) for Ackermann model
    %
    % INPUTS:
    %   sock      - TCP socket connection to robot
    %   v         - Linear velocity [m/s]
    %   omega     - Angular velocity [rad/s]
    %   wheelbase - Robot wheelbase [m]
    
    % Convert angular velocity to steering angle using Ackermann model
    if abs(v) > 0.05  % Moving forward/backward
        steering_angle = atan(wheelbase * omega / v);
    else  % Low speed or stationary
        % Use proportional steering when stopped
        steering_angle = omega * 0.3;
    end
    
    % Clamp steering angle to physical limits
    max_steering = deg2rad(30);  % 30 degrees max
    steering_angle = max(min(steering_angle, max_steering), -max_steering);
    
    % Send command in format expected by robot: "linear_vel,steering_angle\n"
    cmd = sprintf("%.4f,%.4f\n", v, steering_angle);
    write(sock, cmd);
    
    % Read response if available (non-blocking)
    if sock.NumBytesAvailable > 0
        response = read(sock, sock.NumBytesAvailable, "string");
    end
end

function stopRobot(sock, wheelbase)
    % STOPROBOT - Send stop command to robot and straighten wheels
    % Sends multiple stop commands to ensure robot stops
    
    for i = 1:5
        sendRobotCmd(sock, 0, 0, wheelbase);
        pause(0.05);
    end
end

%% ======================= COORDINATE TRANSFORM FUNCTIONS =======================
function [toMazeX, toMazeY] = get_transform_functions(calib)
    % GET_TRANSFORM_FUNCTIONS - Get coordinate transform functions based on calibration
    % Returns function handles to convert MoCap (X,Z) to Maze (X,Y)
    %
    % The transform number is determined during calibration by driving forward
    % and seeing which transform gives positive dx and near-zero dy
    
    switch calib.transform
        case 1  % x_maze = +Z, y_maze = +X
            toMazeX = @(xg, zg) zg - calib.origin_z;
            toMazeY = @(xg, zg) xg - calib.origin_x;
        case 2  % x_maze = -Z, y_maze = +X
            toMazeX = @(xg, zg) -(zg - calib.origin_z);
            toMazeY = @(xg, zg) xg - calib.origin_x;
        case 3  % x_maze = +X, y_maze = +Z
            toMazeX = @(xg, zg) xg - calib.origin_x;
            toMazeY = @(xg, zg) zg - calib.origin_z;
        case 4  % x_maze = +X, y_maze = -Z (most common for our setup)
            toMazeX = @(xg, zg) xg - calib.origin_x;
            toMazeY = @(xg, zg) -(zg - calib.origin_z);
        otherwise
            error('Invalid transform number: %d', calib.transform);
    end
end

function [x_maze, y_maze, theta_maze] = transform_pose(pose, calib)
    % TRANSFORM_POSE - Transform MoCap pose to maze coordinates
    % Returns maze position and heading
    
    [toMazeX, toMazeY] = get_transform_functions(calib);
    
    % Transform position
    x_maze = toMazeX(pose.pos(1), pose.pos(3));
    y_maze = toMazeY(pose.pos(1), pose.pos(3));
    
    % Transform heading (subtract origin yaw)
    theta_maze = wrapToPi(pose.yaw - calib.origin_yaw);
end

%% ======================= PATH BUILDING FUNCTIONS =======================
function [ref, curvature] = build_dubins_path(wps_2d, R_min, step, v_nom, obs, obsR)
    % BUILD_DUBINS_PATH - Generate collision-free Dubins path through waypoints
    % 
    % INPUTS:
    %   wps_2d  - Nx2 waypoint positions [x, y]
    %   R_min   - Minimum turning radius [m]
    %   step    - Interpolation step size [m]
    %   v_nom   - Nominal velocity [m/s] for time calculation
    %   obs     - Mx2 obstacle positions [x, y] (optional)
    %   obsR    - Obstacle radius [m] (optional)
    %
    % OUTPUTS:
    %   ref       - Structure with fields:
    %               .x, .y, .theta (path coordinates and heading)
    %               .s (arc length along path)
    %               .t (time along path at nominal velocity)
    %   curvature - Curvature at each point [1/m]
    
    % Check inputs
    if nargin < 5
        obs = [];
        obsR = 0;
    end
    
    % Helper function to get direction angle between two points
    dirAngle = @(p1,p2) atan2(p2(2)-p1(2), p2(1)-p1(1));
    
    % Build waypoints with headings
    % Each waypoint's heading points toward the NEXT waypoint
    wps = zeros(size(wps_2d,1), 3);
    for k = 1:size(wps_2d,1)
        wps(k,1:2) = wps_2d(k,:);
        if k < size(wps_2d,1)
            % Point toward next waypoint
            wps(k,3) = dirAngle(wps_2d(k,:), wps_2d(k+1,:));
        else
            % Last waypoint: keep previous heading
            wps(k,3) = wps(k-1,3);
        end
    end
    
    % Connect waypoints with Dubins curves
    dubConn = dubinsConnection;
    dubConn.MinTurningRadius = R_min;
    fullPath = [];
    
    for k = 1:(size(wps,1)-1)
        % Get all possible Dubins paths between waypoints
        [segList, costs] = connect(dubConn, wps(k,:), wps(k+1,:), "PathSegments", "all");
        
        % Choose shortest path (could add collision checking here)
        [~, idx] = min(costs);
        
        % Interpolate the path
        L = segList{idx}.Length;
        pts = interpolate(segList{idx}, 0:step:L);
        
        % Check for collisions if obstacles provided
        if ~isempty(obs) && obsR > 0
            for i = 1:size(obs,1)
                dists = vecnorm(pts(:,1:2) - obs(i,:), 2, 2);
                if any(dists < obsR)
                    warning('Path collides with obstacle %d at (%.2f, %.2f)', i, obs(i,1), obs(i,2));
                end
            end
        end
        
        fullPath = [fullPath; pts];
    end
    
    % Remove duplicate points at waypoint connections
    tol = 1e-6;
    keep = [true; vecnorm(diff(fullPath(:,1:2)), 2, 2) > tol];
    fullPath = fullPath(keep, :);
    
    % Extract coordinates
    x = fullPath(:,1);
    y = fullPath(:,2);
    theta = fullPath(:,3);
    
    % Compute arc length
    dx = diff(x);
    dy = diff(y);
    ds = sqrt(dx.^2 + dy.^2);
    s = [0; cumsum(ds)];
    
    % Compute time (assuming constant velocity)
    t = s / v_nom;
    
    % Compute curvature (κ = dθ/ds)
    dtheta = diff(theta);
    % Handle angle wrapping
    dtheta = wrapToPi(dtheta);
    
    curvature = zeros(size(theta));
    curvature(2:end) = dtheta ./ max(ds, 1e-6);
    curvature(1) = curvature(2);  % Copy first valid curvature
    
    % Package output
    ref.x = x;
    ref.y = y;
    ref.theta = theta;
    ref.s = s;
    ref.t = t;
end

%% ======================= EKF FUNCTIONS =======================
function [x_pred, P_pred] = ekf_predict(x, P, v_cmd, omega_cmd, dt, Q, tau_v, tau_omega)
    % EKF_PREDICT - Extended Kalman Filter prediction step
    % State vector: [x, y, theta, v, omega]'
    
    % Ensure column vectors
    x = x(:);
    
    % Extract states
    theta = x(3);
    v = x(4);
    omega = x(5);
    
    % Continuous-time dynamics: xdot = f(x, u)
    f = [v * cos(theta);           % xdot
         v * sin(theta);           % ydot  
         omega;                    % thetadot
         (v_cmd - v) / tau_v;      % vdot (first-order lag)
         (omega_cmd - omega) / tau_omega]; % omegadot (first-order lag)
    
    % Discrete-time prediction
    x_pred = x + f * dt;
    x_pred(3) = wrapToPi(x_pred(3));  % Wrap angle
    
    % Jacobian of dynamics
    F = [0, 0, -v*sin(theta), cos(theta), 0;
         0, 0,  v*cos(theta), sin(theta), 0;
         0, 0,  0,            0,          1;
         0, 0,  0,           -1/tau_v,    0;
         0, 0,  0,            0,         -1/tau_omega];
    
    % Discrete-time state transition matrix
    A = eye(5) + F * dt;
    
    % Predict covariance
    P_pred = A * P * A' + Q * dt;
end

function [x_upd, P_upd] = ekf_update(x, P, z, R)
    % EKF_UPDATE - Extended Kalman Filter measurement update
    % Measurement vector: [x, y, theta]'
    
    % Ensure column vectors
    x = x(:);
    z = z(:);
    
    % Measurement model: z = h(x) = [x; y; theta]
    H = [1, 0, 0, 0, 0;   % x measurement
         0, 1, 0, 0, 0;   % y measurement
         0, 0, 1, 0, 0];  % theta measurement
    
    % Predicted measurement
    z_pred = H * x;
    
    % Innovation (measurement residual)
    y = z - z_pred;
    y(3) = wrapToPi(y(3));  % Wrap angle difference
    
    % Innovation covariance
    S = H * P * H' + R;
    
    % Kalman gain
    K = P * H' / S;
    
    % Update state
    x_upd = x + K * y;
    x_upd(3) = wrapToPi(x_upd(3));  % Wrap angle
    
    % Update covariance (Joseph form for numerical stability)
    I_KH = eye(5) - K * H;
    P_upd = I_KH * P * I_KH' + K * R * K';
end

%% ======================= CONTROL FUNCTIONS =======================
function [v_cmd, omega_cmd, e_y, e_theta, idx_ref] = compute_control(x_pos, y_pos, theta_pos, ref, K_ey, K_etheta, lookahead, v_nom)
    % COMPUTE_CONTROL - Compute path following control using lateral error
    %
    % CONTROL LAW:
    %   Find closest point on path, then look ahead
    %   Compute lateral and heading errors
    %   Apply proportional control with negative feedback
    %
    % NOTE: This assumes Y coordinate may need flipping (handled in caller)
    
    % Find closest point on reference path
    dists = sqrt((ref.x - x_pos).^2 + (ref.y - y_pos).^2);
    [~, idx_closest] = min(dists);
    
    % Current arc length
    s_current = ref.s(idx_closest);
    
    % Look ahead
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
    
    % Lateral error (cross-track error)
    e_y = -dx * sin(theta_ref) + dy * cos(theta_ref);
    
    % Heading error
    e_theta = wrapToPi(theta_pos - theta_ref);
    
    % Control law with negative feedback
    v_cmd = v_nom;  % Constant forward speed
    omega_cmd = -(K_ey * e_y + K_etheta * e_theta);
    
    % Clamp angular velocity
    omega_max = 1.0;  % rad/s
    omega_cmd = max(min(omega_cmd, omega_max), -omega_max);
end

%% ======================= VISUALIZATION FUNCTIONS =======================
function setup_figure_fullscreen()
    % SETUP_FIGURE_FULLSCREEN - Create fullscreen figure for visualization
    figure('Units', 'normalized', 'Position', [0 0 1 1]);
end

function plot_arena(bounds)
    % PLOT_ARENA - Draw arena boundaries
    % bounds = [xmin xmax ymin ymax]
    
    plot([bounds(1) bounds(2) bounds(2) bounds(1) bounds(1)], ...
         [bounds(3) bounds(3) bounds(4) bounds(4) bounds(3)], ...
         'k', 'LineWidth', 3);
end

function plot_obstacles(obs, obsR)
    % PLOT_OBSTACLES - Draw circular obstacles
    % Note: Y coordinate is flipped for visualization
    
    if isempty(obs)
        return;
    end
    
    scatter(obs(:,1), -obs(:,2), 150, 'k', 'filled');
    for i = 1:size(obs, 1)
        viscircles([obs(i,1), -obs(i,2)], obsR, ...
                   'Color', [0.8 0 0], 'LineStyle', '--', 'LineWidth', 2);
    end
end

function plot_path(ref, wps_2d)
    % PLOT_PATH - Draw reference path and waypoints
    % Note: Y coordinate is flipped for visualization
    
    % Path
    plot(ref.x, -ref.y, 'r--', 'LineWidth', 3);
    
    % Waypoints
    plot(wps_2d(:,1), -wps_2d(:,2), 'mo', 'MarkerSize', 15, 'MarkerFaceColor', 'm');
    
    % Start position
    plot(ref.x(1), -ref.y(1), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
end

%% ======================= UTILITY FUNCTIONS =======================
function angle = wrapToPi(angle)
    % WRAPTOPI - Wrap angle to [-pi, pi]
    angle = mod(angle + pi, 2*pi) - pi;
end

function print_status(t, x_pos, y_pos, theta_pos, s_current, s_total, e_y, e_theta)
    % PRINT_STATUS - Display current robot status
    fprintf('\r[%.1fs] Pos:(%.2f,%.2f) θ:%.0f° | Progress:%.1f/%.1fm | Errors: e_y=%.3fm e_θ=%.0f°', ...
            t, x_pos, y_pos, rad2deg(theta_pos), s_current, s_total, e_y, rad2deg(e_theta));
end
