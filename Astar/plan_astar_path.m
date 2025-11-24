function waypoints = plan_astar_path(start_pos, goal_pos, obstacles, obsR, cfg)
    %% PLAN_ASTAR_PATH - Generate waypoints using A* grid-based planning
    
    res = cfg.grid_resolution;
    inflation = cfg.planning_inflation;
    bounds = cfg.arena_bounds;
    
    xmin = bounds(1); xmax = bounds(2);
    ymin = bounds(3); ymax = bounds(4);
    
    width = xmax - xmin;
    height = ymax - ymin;
    
    map = binaryOccupancyMap(width, height, 1/res);
    map.GridLocationInWorld = [xmin, ymin];
    
    inflate_radius = obsR + inflation;
    
    % Build occupancy grid
    [X, Y] = meshgrid(xmin:res:xmax, ymin:res:ymax);
    occ = zeros(size(X));
    for i = 1:size(obstacles, 1)
        dist = sqrt((X - obstacles(i,1)).^2 + (Y - obstacles(i,2)).^2);
        occ = occ | (dist <= inflate_radius);
    end
    
    % Set occupancy
    coords = [X(occ==1), Y(occ==1)];
    if ~isempty(coords)
        setOccupancy(map, coords, 1);
    end
    
    % CRITICAL: Force clear cells at start and goal positions
    % This is essential for path planning to work
    setOccupancy(map, start_pos, 0, 'world');
    setOccupancy(map, goal_pos, 0, 'world');
    
    % Also clear a small radius around start and goal (3 cells)
    clear_radius = res * 3;
    for angle = 0:30:359
        for r = res:res:clear_radius
            dx = r * cosd(angle);
            dy = r * sind(angle);
            
            % Clear around start
            pos = start_pos + [dx, dy];
            if pos(1) >= xmin && pos(1) <= xmax && pos(2) >= ymin && pos(2) <= ymax
                % Check not inside actual obstacle
                dists = sqrt((obstacles(:,1) - pos(1)).^2 + (obstacles(:,2) - pos(2)).^2);
                if min(dists) > obsR - 0.05  % Small tolerance
                    setOccupancy(map, pos, 0, 'world');
                end
            end
            
            % Clear around goal
            pos = goal_pos + [dx, dy];
            if pos(1) >= xmin && pos(1) <= xmax && pos(2) >= ymin && pos(2) <= ymax
                dists = sqrt((obstacles(:,1) - pos(1)).^2 + (obstacles(:,2) - pos(2)).^2);
                if min(dists) > obsR - 0.05
                    setOccupancy(map, pos, 0, 'world');
                end
            end
        end
    end
    
    % Verify positions are valid
    if start_pos(1) < xmin || start_pos(1) > xmax || ...
       start_pos(2) < ymin || start_pos(2) > ymax
        error('Start position is outside map bounds.');
    end
    if goal_pos(1) < xmin || goal_pos(1) > xmax || ...
       goal_pos(2) < ymin || goal_pos(2) > ymax
        error('Goal position is outside map bounds.');
    end
    
    % Check for actual obstacle collision
    start_dists = sqrt((obstacles(:,1) - start_pos(1)).^2 + (obstacles(:,2) - start_pos(2)).^2);
    if min(start_dists) < obsR - 0.05
        error('Start position (%.2f, %.2f) is inside obstacle!', start_pos(1), start_pos(2));
    end
    
    goal_dists = sqrt((obstacles(:,1) - goal_pos(1)).^2 + (obstacles(:,2) - goal_pos(2)).^2);
    if min(goal_dists) < obsR - 0.05
        error('Goal position (%.2f, %.2f) is inside obstacle!', goal_pos(1), goal_pos(2));
    end
    
    % Create planner and plan
    planner = plannerAStarGrid(map);
    
    % Plan in world coordinates
    [path, solnInfo] = plan(planner, start_pos, goal_pos, 'world');
    
    % Check if path was found
    foundPath = false;
    if isstruct(solnInfo) && isfield(solnInfo, 'IsPathFound')
        foundPath = logical(solnInfo.IsPathFound);
    elseif istable(solnInfo) && any(strcmp(solnInfo.Properties.VariableNames,'IsPathFound'))
        foundPath = logical(solnInfo.IsPathFound(1));
    else
        foundPath = ~isempty(path) && size(path,1) > 0;
    end
    
    if ~foundPath || isempty(path)
        error('A* could not find a path from start to goal!');
    end
    
    % Simplify path with moderate approach
    waypoints = moderate_simplify(path, cfg.R_min);
end

function simplified = moderate_simplify(path, R_min)
    % Moderate simplification - keep enough waypoints for Dubins
    
    if size(path,1) <= 2
        simplified = path;
        return;
    end
    
    % First pass: remove strictly collinear points
    kept = [path(1,:)];
    
    for i = 2:(size(path,1)-1)
        v1 = path(i,:) - kept(end,:);
        v2 = path(i+1,:) - path(i,:);
        
        if norm(v1) > 1e-6 && norm(v2) > 1e-6
            v1_norm = v1 / norm(v1);
            v2_norm = v2 / norm(v2);
            angle_change = acos(max(-1, min(1, dot(v1_norm, v2_norm))));
            
            % Keep waypoint if direction changes significantly
            if angle_change > deg2rad(15)  % More sensitive to direction changes
                kept = [kept; path(i,:)];
            end
        end
    end
    
    % Always keep goal
    kept = [kept; path(end,:)];
    
    % Second pass: ensure minimum spacing for Dubins curves
    simplified = [kept(1,:)];
    
    for i = 2:size(kept,1)
        dist = norm(kept(i,:) - simplified(end,:));
        % Increased minimum spacing to give Dubins more room
        if dist > 0.6 * R_min || i == size(kept,1)
            simplified = [simplified; kept(i,:)];
        end
    end
end
