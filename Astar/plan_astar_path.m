function waypoints = plan_astar_path(start_pos, goal_pos, obstacles, obsR, cfg)
    %% PLAN_ASTAR_PATH - Generate waypoints using A* grid-based planning
    
    res = cfg.grid_resolution;
    inflation = cfg.planning_inflation;
    bounds = cfg.arena_bounds;  % [xmin xmax ymin ymax]
    
    xmin = bounds(1); xmax = bounds(2);
    ymin = bounds(3); ymax = bounds(4);
    
    width = xmax - xmin;
    height = ymax - ymin;
    
    map = binaryOccupancyMap(width, height, 1/res);
    map.GridLocationInWorld = [xmin, ymin];
    
    inflate_radius = obsR + inflation;
    
    % Build a grid of world coordinates once
    [X, Y] = meshgrid(xmin:res:xmax, ymin:res:ymax);
    
    occ = zeros(size(X));
    for i = 1:size(obstacles, 1)
        dist = sqrt((X - obstacles(i,1)).^2 + (Y - obstacles(i,2)).^2);
        occ = occ | (dist <= inflate_radius);
    end
    
    % Set occupancy using vectorized call
    coords = [X(occ==1), Y(occ==1)];
    if ~isempty(coords)
        setOccupancy(map, coords, 1);
    end
    
    % CRITICAL: Clear small regions around start and goal
    % This ensures paths can be found even near obstacles
    clearance_radius = res * 2;  % Clear 2 grid cells around start/goal
    
    % Clear start region
    for dx = -clearance_radius:res:clearance_radius
        for dy = -clearance_radius:res:clearance_radius
            pos = start_pos + [dx, dy];
            if pos(1) >= xmin && pos(1) <= xmax && pos(2) >= ymin && pos(2) <= ymax
                % Only clear if not inside actual obstacle
                min_dist = min(sqrt((obstacles(:,1) - pos(1)).^2 + (obstacles(:,2) - pos(2)).^2));
                if min_dist > obsR
                    setOccupancy(map, pos, 0, 'world');
                end
            end
        end
    end
    
    % Clear goal region
    for dx = -clearance_radius:res:clearance_radius
        for dy = -clearance_radius:res:clearance_radius
            pos = goal_pos + [dx, dy];
            if pos(1) >= xmin && pos(1) <= xmax && pos(2) >= ymin && pos(2) <= ymax
                % Only clear if not inside actual obstacle
                min_dist = min(sqrt((obstacles(:,1) - pos(1)).^2 + (obstacles(:,2) - pos(2)).^2));
                if min_dist > obsR
                    setOccupancy(map, pos, 0, 'world');
                end
            end
        end
    end
    
    % Verify start and goal are valid
    if start_pos(1) < xmin || start_pos(1) > xmax || ...
       start_pos(2) < ymin || start_pos(2) > ymax
        error('Start position is outside map bounds.');
    end
    if goal_pos(1) < xmin || goal_pos(1) > xmax || ...
       goal_pos(2) < ymin || goal_pos(2) > ymax
        error('Goal position is outside map bounds.');
    end
    
    % Check if start/goal are inside actual obstacles
    start_dists = sqrt((obstacles(:,1) - start_pos(1)).^2 + (obstacles(:,2) - start_pos(2)).^2);
    if min(start_dists) < obsR
        error('Start position (%.2f, %.2f) is inside obstacle!', start_pos(1), start_pos(2));
    end
    
    goal_dists = sqrt((obstacles(:,1) - goal_pos(1)).^2 + (obstacles(:,2) - goal_pos(2)).^2);
    if min(goal_dists) < obsR
        error('Goal position (%.2f, %.2f) is inside obstacle!', goal_pos(1), goal_pos(2));
    end
    
    % Create planner and plan
    planner = plannerAStarGrid(map);
    
    % Plan in world coordinates
    [path, solnInfo] = plan(planner, start_pos, goal_pos, 'world');
    
    % Determine whether a path was found (compatible across MATLAB versions)
    foundPath = false;
    if isstruct(solnInfo) && isfield(solnInfo, 'IsPathFound')
        foundPath = logical(solnInfo.IsPathFound);
    elseif istable(solnInfo) && any(strcmp(solnInfo.Properties.VariableNames,'IsPathFound'))
        foundPath = logical(solnInfo.IsPathFound(1));
    else
        % Fallback: if path is nonempty treat as found
        foundPath = ~isempty(path) && size(path,1) > 0;
    end
    
    if ~foundPath
        error('A* could not find a path from start to goal!');
    end
    
    % Simplify path - smart waypoint reduction
    waypoints = simplify_path_smart(path, obstacles, obsR, cfg.R_min);
end

function simplified = simplify_path_smart(path, obstacles, obsR, R_min)
    % Smart path simplification that maintains navigability
    
    if size(path,1) <= 2
        simplified = path;
        return;
    end
    
    simplified = [path(1,:)];  % Always keep start
    
    i = 1;
    while i < size(path,1)
        % Try to skip as many points as possible
        best_skip = i + 1;  % At minimum, go to next point
        
        for j = (i+2):min(i+20, size(path,1))  % Look ahead up to 20 points
            % Check if we can go directly from current to j
            if can_connect(path(i,:), path(j,:), obstacles, obsR)
                best_skip = j;
            else
                break;  % Can't skip further
            end
        end
        
        % Add the furthest reachable point
        simplified = [simplified; path(best_skip,:)];
        i = best_skip;
    end
    
    % Always ensure goal is included
    if ~isequal(simplified(end,:), path(end,:))
        simplified = [simplified; path(end,:)];
    end
    
    % Further reduce by removing collinear points
    final = [simplified(1,:)];
    for i = 2:(size(simplified,1)-1)
        v1 = simplified(i,:) - final(end,:);
        v2 = simplified(i+1,:) - simplified(i,:);
        
        if norm(v1) < 1e-6 || norm(v2) < 1e-6
            continue;
        end
        
        v1 = v1 / norm(v1);
        v2 = v2 / norm(v2);
        
        angle_change = acos(max(-1, min(1, dot(v1, v2))));
        
        % Keep point if significant direction change
        if angle_change > deg2rad(15)
            final = [final; simplified(i,:)];
        end
    end
    
    final = [final; simplified(end,:)];
    
    % Ensure minimum spacing for Dubins curves
    spaced = [final(1,:)];
    for i = 2:size(final,1)
        if norm(final(i,:) - spaced(end,:)) > 0.5 * R_min || i == size(final,1)
            spaced = [spaced; final(i,:)];
        end
    end
    
    simplified = spaced;
end

function can_connect = can_connect(p1, p2, obstacles, obsR)
    % Check if straight line from p1 to p2 is collision-free
    
    n_checks = ceil(norm(p2 - p1) / 0.1);  % Check every 0.1m
    
    for alpha = linspace(0, 1, n_checks)
        point = p1 + alpha * (p2 - p1);
        
        % Check distance to all obstacles
        dists = sqrt((obstacles(:,1) - point(1)).^2 + (obstacles(:,2) - point(2)).^2);
        if min(dists) <= obsR + 0.1  % Small margin
            can_connect = false;
            return;
        end
    end
    
    can_connect = true;
end
