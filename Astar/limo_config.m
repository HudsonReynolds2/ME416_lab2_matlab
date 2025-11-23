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
    cfg.grid_resolution = 0.5;           % [m] Grid cell size for A* (coarser = smoother)
    cfg.planning_inflation = 0.50;       % [m] Obstacle inflation for path planning
    
    %% ======================= CONTROL GAINS =======================
    cfg.K_ey = 8.0;                      % Cross-track error gain
    cfg.K_etheta = 3.0;                  % Heading error gain
    cfg.K_ev = 0.5;                      % Velocity error gain (unused in current formulation)
    cfg.LOOKAHEAD = 0.40;                % [m] Lookahead distance for reference point
    
    %% ======================= OBSTACLE PARAMETERS =======================
    cfg.obsR = 0.30;                     % [m] Actual obstacle radius
    cfg.collision_dist = 0.35;           % [m] Distance to consider collision (obsR + margin)
    cfg.safe_dist = 0.80;                % [m] Distance to trigger replanning
    
    %% ======================= MAZE DEFINITION =======================
    % MOAP Lab Maze Layout - Obstacle positions in maze coordinates
    % After calibration, (0,0) is maze start, coordinates match physical layout
    cfg.obs = [
        1.5 0.0; 1.5 0.5; 1.5 1.0; 1.5 1.5; 1.5 2.0;         % Left wall
        3.3 3.0; 3.3 3.5; 3.3 4.0; 3.3 4.5; 3.3 5.0; 3.3 5.5  % Right wall
    ];
    
    % Start and goal positions in maze coordinates
    cfg.start_pos = [0.0, 0.0];          % Maze origin (set by calibration)
    cfg.goal_pos = [5.0, 4.5];           % End of maze course
    
    cfg.courseName = "MOAP Lab Course 1";
    
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
    
    %% ======================= VISUALIZATION SETTINGS =======================
    % Arena bounds for plotting [xmin xmax ymin ymax]
    % All coordinates in maze frame (no flipping)
    cfg.arena_bounds = [-0.5 5.5 -1.0 6.0];
    
    %% ======================= TIMING PARAMETERS =======================
    cfg.MIN_REPLAN_INTERVAL = 1.0;      % [s] Minimum time between replanning
    cfg.TIMEOUT = 60.0;                 % [s] Maximum run time
    
    %% ======================= CALIBRATION PARAMETERS =======================
    cfg.calib_drive_speed = 0.20;       % [m/s] Speed for calibration drive
    cfg.calib_drive_time = 2.5;         % [s] Time to drive forward during calibration
    cfg.calib_return_extra = 0.05;      % [m] Extra distance to reverse (to account for lag)
    
end
