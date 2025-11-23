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

    % CRITICAL FIX: Force start and goal cells to be free
    % Only clear the exact cells, not large regions
    setOccupancy(map, start_pos, 0, 'world');
    setOccupancy(map, goal_pos, 0, 'world');

    % Check start/goal are inside map bounds
    if start_pos(1) < xmin || start_pos(1) > xmax || ...
       start_pos(2) < ymin || start_pos(2) > ymax
        error('Start position is outside map bounds.');
    end
    if goal_pos(1) < xmin || goal_pos(1) > xmax || ...
       goal_pos(2) < ymin || goal_pos(2) > ymax
        error('Goal position is outside map bounds.');
    end

    if getOccupancy(map, start_pos, 'world')
        % Double-check with actual obstacle distances (not inflated grid)
        start_dists = sqrt((obstacles(:,1) - start_pos(1)).^2 + (obstacles(:,2) - start_pos(2)).^2);
        if min(start_dists) < obsR
            error('Start position (%.2f, %.2f) is inside obstacle!', start_pos(1), start_pos(2));
        end
        % Otherwise it's just in the inflated region, which is OK
    end
    if getOccupancy(map, goal_pos, 'world')
        % Double-check with actual obstacle distances
        goal_dists = sqrt((obstacles(:,1) - goal_pos(1)).^2 + (obstacles(:,2) - goal_pos(2)).^2);
        if min(goal_dists) < obsR
            error('Goal position (%.2f, %.2f) is inside obstacle!', goal_pos(1), goal_pos(2));
        end
        % Otherwise it's just in the inflated region, which is OK
    end

    planner = plannerAStarGrid(map);

    % Plan in world coordinates — this avoids needing integer grid indices
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

    % Simplify A* path - remove collinear points
    % A* gives us grid-aligned steps, we want just the corner points
    simplified = [path(1,:)];
    
    for i = 2:(size(path,1)-1)
        % Vector from last kept point to current point
        v1 = path(i,:) - simplified(end,:);
        % Vector from current to next
        v2 = path(i+1,:) - path(i,:);
        
        if norm(v1) < 1e-6 || norm(v2) < 1e-6
            continue;
        end
        
        % Normalize
        v1 = v1 / norm(v1);
        v2 = v2 / norm(v2);
        
        % Calculate angle change properly
        dot_prod = dot(v1, v2);
        dot_prod = max(-1, min(1, dot_prod));  % Clamp to avoid numerical issues
        angle_change = acos(dot_prod);
        
        % If direction changes significantly, keep this point
        % Increased threshold to 15 degrees to filter out more grid artifacts
        if angle_change > deg2rad(15)
            simplified = [simplified; path(i,:)];
        end
    end
    
    simplified = [simplified; path(end,:)];
    
    % Additional pass: ensure minimum spacing between waypoints for Dubins
    min_spacing = cfg.grid_resolution * 1.2;  % At least 1.2 grid cells apart
    final_waypoints = [simplified(1,:)];
    
    for i = 2:size(simplified,1)
        dist_from_last = norm(simplified(i,:) - final_waypoints(end,:));
        if dist_from_last >= min_spacing || i == size(simplified,1)
            final_waypoints = [final_waypoints; simplified(i,:)];
        end
    end
    
    waypoints = final_waypoints;
end