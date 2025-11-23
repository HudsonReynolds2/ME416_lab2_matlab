function cfg = limo_config()
    %% LIMO_CONFIG - Centralized configuration for ALL Limo robot scripts
    % Returns a struct with all parameters needed by:
    %   - calibrate_limo.m
    %   - manual_drive_logger.m  
    %   - ekf_lqr_dubins.m
    %   - test_path_visualization.m
    
    %% ======================= ROBOT PHYSICAL PARAMETERS =======================
    cfg.WHEELBASE = 0.2;                 % [m] Limo wheelbase
    cfg.MAX_STEERING_DEG = 30;           % [deg] Maximum steering angle
    cfg.MAX_STEERING_RAD = deg2rad(30);  % [rad] Maximum steering angle
    cfg.MAX_SPEED = 0.2;                 % [m/s] Robot speed limit (enforced by Python)
    
    %% ======================= CONTROL PARAMETERS =======================
    cfg.v_nom = 0.30;                    % [m/s] Nominal forward speed
    cfg.v_manual = 0.30;                 % [m/s] Manual drive speed (same as auto)
    cfg.omega_max = 1.0;                 % [rad/s] Maximum angular velocity
    cfg.dt_control = 0.05;               % [s] Control loop period (20 Hz)
    
    %% ======================= PATH PLANNING PARAMETERS =======================
    cfg.R_min = 0.30;                    % [m] Minimum turning radius for Dubins
    cfg.step = 0.05;                     % [m] Dubins path interpolation step
    
    % A* grid-based planning parameters
    cfg.grid_resolution = 0.30;          % [m] Finer grid for narrow passages
    cfg.planning_inflation = 0.25;       % [m] Reduced inflation - robot handles tight spaces
    
    %% ======================= CONTROL GAINS =======================
    cfg.K_ey = 8.0;                      % Cross-track error gain
    cfg.K_etheta = 3.0;                  % Heading error gain
    cfg.K_ev = 0.5;                      % Velocity error gain (unused in current formulation)
    cfg.LOOKAHEAD = 0.40;                % [m] Lookahead distance for reference point
    
    %% ======================= OBSTACLE PARAMETERS =======================
    cfg.obsR = 0.30;                     % [m] Actual obstacle radius (physical size)
    cfg.collision_dist = 0.35;           % [m] Distance to consider collision (obsR + margin)
    cfg.safe_dist = 0.80;                % [m] Distance to trigger replanning
    
    %% ======================= MAZE DEFINITION =======================
    % Multiple maze courses - select with cfg.maze_select (1, 2, or 3)
    
    % Maze 1: MOAP Lab Course 1 (original)
    cfg.mazes(1).name = "MOAP Lab Course 1";
    cfg.mazes(1).obs = [
        1.5 0.0; 1.5 0.5; 1.5 1.0; 1.5 1.5; 1.5 2.0;         % Left wall
        3.3 3.0; 3.3 3.5; 3.3 4.0; 3.3 4.5; 3.3 5.0; 3.3 5.5  % Right wall
    ];
    cfg.mazes(1).start_pos = [0.0, 0.0];
    cfg.mazes(1).goal_pos = [5.0, 4.5];
    cfg.mazes(1).arena_bounds = [-0.5 5.5 -1.0 6.0];
    
    % Maze 2: 3x3 Grid with proper boundary walls
    cfg.mazes(2).name = "3x3 Grid Maze";
    interior_obs = [
        1.0 1.0; 2.5 1.0; 4.0 1.0;
        0.0 2.5; 1.5 2.5; 3.0 2.5;
        1.0 4.0; 2.5 4.0; 4.0 4.0
    ];
    % Dense boundary walls outside playable area
    wall_spacing = 0.2;
    x_wall = (-0.5:wall_spacing:5.5)';
    y_wall = (-0.5:wall_spacing:5.0)';
    bottom = [x_wall, ones(size(x_wall))*-0.5];
    top = [x_wall, ones(size(x_wall))*5.0];
    left = [ones(size(y_wall))*-0.5, y_wall];
    right = [ones(size(y_wall))*5.5, y_wall];
    cfg.mazes(2).obs = [interior_obs; bottom; top; left; right];
    cfg.mazes(2).start_pos = [0.3, 0.3];
    cfg.mazes(2).goal_pos = [4.7, 4.2];
    cfg.mazes(2).arena_bounds = [-0.7 5.7 -0.7 5.2];
    
    % Maze 3: Complex Path with boundary walls
    cfg.mazes(3).name = "Complex Path Maze";
    interior_obs = [
        0.0 1.0; 0.5 1.0; 1.0 1.0; 1.0 1.5; 1.0 2.0; 1.5 2.0; 2.0 2.0;
        2.5 2.0; 2.5 2.5; 2.5 3.0; 2.5 3.5; 3.0 3.5; 3.5 3.5; 4.0 3.5;
        4.0 3.0; 4.0 2.5; 4.0 2.0; 4.0 1.5; 4.0 1.0; 0.0 3.5; 0.5 3.5;
        1.0 3.5; 2.5 0.0; 2.5 0.5
    ];
    % Dense boundary walls
    wall_spacing = 0.2;
    x_wall = (-0.7:wall_spacing:5.2)';
    y_wall = (-0.7:wall_spacing:4.7)';
    bottom = [x_wall, ones(size(x_wall))*-0.7];
    top = [x_wall, ones(size(x_wall))*4.7];
    left = [ones(size(y_wall))*-0.7, y_wall];
    right = [ones(size(y_wall))*5.2, y_wall];
    cfg.mazes(3).obs = [interior_obs; bottom; top; left; right];
    cfg.mazes(3).start_pos = [0.3, 0.3];
    cfg.mazes(3).goal_pos = [0.3, 2.0];
    cfg.mazes(3).arena_bounds = [-0.9 5.4 -0.9 4.9];
    
    % Default maze selection
    cfg.maze_select = 1;  % Default to maze 1
    
    % Set current maze parameters (for backward compatibility)
    cfg.obs = cfg.mazes(cfg.maze_select).obs;
    cfg.start_pos = cfg.mazes(cfg.maze_select).start_pos;
    cfg.goal_pos = cfg.mazes(cfg.maze_select).goal_pos;
    cfg.arena_bounds = cfg.mazes(cfg.maze_select).arena_bounds;
    cfg.courseName = cfg.mazes(cfg.maze_select).name;
    
    %% ======================= EKF PARAMETERS =======================
    cfg.TAU_V = 0.15;                    % [s] Velocity time constant
    cfg.TAU_OMEGA = 0.15;                % [s] Angular velocity time constant
    
    % Process noise covariance
    cfg.Q_pos = 0.01^2;                  % Position noise
    cfg.Q_theta = (2*pi/180)^2;          % Heading noise (2 deg)
    cfg.Q_v = 0.05^2;                    % Velocity noise
    cfg.Q_omega = (5*pi/180)^2;          % Angular velocity noise (5 deg/s)
    cfg.Q = diag([cfg.Q_pos, cfg.Q_pos, cfg.Q_theta, cfg.Q_v, cfg.Q_omega]);
    
    % Measurement noise covariance
    cfg.R_pos = 0.002^2;                 % OptiTrack position accuracy
    cfg.R_theta = (1*pi/180)^2;          % OptiTrack angle accuracy (1 deg)
    cfg.R = diag([cfg.R_pos, cfg.R_pos, cfg.R_theta]);
    
    % Initial covariance
    cfg.P0 = diag([0.01^2, 0.01^2, (2*pi/180)^2, 0.1^2, (10*pi/180)^2]);
    
    %% ======================= CONNECTION PARAMETERS =======================
    cfg.broker = "mqtt://rasticvm.lan";
    cfg.mqtt_port = 1883;
    cfg.topic_pose = "rb/limo809";      % OptiTrack publishes pose here
    cfg.LIMO_IP = "192.168.1.172";      % Robot's IP address
    cfg.LIMO_PORT = 12345;              % TCP port for robot control
    
    %% ======================= FILE NAMES =======================
    cfg.CALIB_FILE = 'limo_calibration.mat';
    
    %% ======================= TIMING PARAMETERS =======================
    cfg.MIN_REPLAN_INTERVAL = 1.0;      % [s] Minimum time between replanning
    cfg.TIMEOUT = 60.0;                 % [s] Maximum run time
    
    %% ======================= CALIBRATION PARAMETERS =======================
    cfg.calib_drive_speed = 0.20;       % [m/s] Speed for calibration drive
    cfg.calib_drive_time = 2.5;         % [s] Time to drive forward during calibration
    cfg.calib_return_extra = 0.05;      % [m] Extra distance to reverse (to account for lag)
    
end
