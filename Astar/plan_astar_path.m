function waypoints = plan_astar_path(start_pos, goal_pos, obstacles, obsR, cfg)
    %% PLAN_ASTAR_PATH - Generate waypoints using A* grid-based planning
    % 
    % INPUTS:
    %   start_pos   - [x, y] start position in maze coordinates
    %   goal_pos    - [x, y] goal position in maze coordinates
    %   obstacles   - Nx2 array of obstacle positions
    %   obsR        - Obstacle radius [m]
    %   cfg         - Configuration struct with grid_resolution, planning_inflation, arena_bounds
    %
    % OUTPUTS:
    %   waypoints   - Mx2 array of waypoint positions [x, y] from start to goal
    
    fprintf('\n=== A* PATH PLANNING ===\n');
    
    % Extract parameters
    res = cfg.grid_resolution;
    inflation = cfg.planning_inflation;
    bounds = cfg.arena_bounds;  % [xmin xmax ymin ymax]
    
    % Create occupancy grid
    xmin = bounds(1); xmax = bounds(2);
    ymin = bounds(3); ymax = bounds(4);
    
    width = xmax - xmin;
    height = ymax - ymin;
    
    % Create binary occupancy map
    map = binaryOccupancyMap(width, height, 1/res);
    map.GridLocationInWorld = [xmin, ymin];
    
    % Inflate obstacles
    inflate_radius = obsR + inflation;
    
    fprintf('Grid: %.1fx%.1f m, resolution: %.2f m\n', width, height, res);
    fprintf('Obstacle inflation: %.2f m\n', inflate_radius);
    
    % Mark obstacle cells
    for i = 1:size(obstacles, 1)
        obs_x = obstacles(i, 1);
        obs_y = obstacles(i, 2);
        
        % Create circular obstacle
        [X, Y] = meshgrid(xmin:res:xmax, ymin:res:ymax);
        dist = sqrt((X - obs_x).^2 + (Y - obs_y).^2);
        obstacle_mask = dist <= inflate_radius;
        
        % Set occupancy
        for ix = 1:size(X, 2)
            for iy = 1:size(Y, 1)
                if obstacle_mask(iy, ix)
                    setOccupancy(map, [X(iy, ix), Y(iy, ix)], 1);
                end
            end
        end
    end
    
    % Verify start and goal are not in obstacles
    if getOccupancy(map, start_pos)
        error('Start position (%.2f, %.2f) is inside obstacle!', start_pos(1), start_pos(2));
    end
    if getOccupancy(map, goal_pos)
        error('Goal position (%.2f, %.2f) is inside obstacle!', goal_pos(1), goal_pos(2));
    end
    
    % Create planner
    planner = plannerAStarGrid(map);
    
    % Plan path
    fprintf('Planning from (%.2f, %.2f) to (%.2f, %.2f)...\n', ...
            start_pos(1), start_pos(2), goal_pos(1), goal_pos(2));
    
    [path, solnInfo] = plan(planner, start_pos, goal_pos);
    
    if ~solnInfo.IsPathFound
        error('A* could not find a path from start to goal!');
    end
    
    fprintf('✓ Path found: %d grid cells\n', size(path, 1));
    
    % Simplify path - remove redundant waypoints
    % Keep points where direction changes significantly
    waypoints = [start_pos];
    
    if size(path, 1) > 2
        for i = 2:(size(path, 1)-1)
            % Direction vectors
            dir1 = path(i, :) - path(i-1, :);
            dir2 = path(i+1, :) - path(i, :);
            
            % Normalize
            dir1 = dir1 / norm(dir1);
            dir2 = dir2 / norm(dir2);
            
            % If direction changes more than 15 degrees, keep waypoint
            angle_change = acos(dot(dir1, dir2));
            if angle_change > deg2rad(15)
                waypoints = [waypoints; path(i, :)];
            end
        end
    end
    
    % Always include goal
    waypoints = [waypoints; goal_pos];
    
    fprintf('✓ Simplified to %d waypoints\n', size(waypoints, 1));
    
    % Print waypoints
    fprintf('\nWaypoints:\n');
    for i = 1:size(waypoints, 1)
        fprintf('  %d: (%.2f, %.2f)\n', i, waypoints(i, 1), waypoints(i, 2));
    end
    
end
