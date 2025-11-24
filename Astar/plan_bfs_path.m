function waypoints = plan_bfs_path(start_pos, goal_pos, obstacles, obsR, cfg)
    %% PLAN_BFS_PATH - Generate waypoints using Breadth-First Search
    
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
    
    coords = [X(occ==1), Y(occ==1)];
    if ~isempty(coords)
        setOccupancy(map, coords, 1);
    end
    
    % Force clear start and goal cells
    setOccupancy(map, start_pos, 0, 'world');
    setOccupancy(map, goal_pos, 0, 'world');
    
    % Clear radius around start and goal
    clear_radius = res * 3;
    for angle = 0:30:359
        for r = res:res:clear_radius
            dx = r * cosd(angle);
            dy = r * sind(angle);
            
            pos = start_pos + [dx, dy];
            if pos(1) >= xmin && pos(1) <= xmax && pos(2) >= ymin && pos(2) <= ymax
                dists = sqrt((obstacles(:,1) - pos(1)).^2 + (obstacles(:,2) - pos(2)).^2);
                if min(dists) > obsR - 0.05
                    setOccupancy(map, pos, 0, 'world');
                end
            end
            
            pos = goal_pos + [dx, dy];
            if pos(1) >= xmin && pos(1) <= xmax && pos(2) >= ymin && pos(2) <= ymax
                dists = sqrt((obstacles(:,1) - pos(1)).^2 + (obstacles(:,2) - pos(2)).^2);
                if min(dists) > obsR - 0.05
                    setOccupancy(map, pos, 0, 'world');
                end
            end
        end
    end
    
    % Convert to grid coordinates
    start_grid = world2grid(map, start_pos);
    goal_grid = world2grid(map, goal_pos);
    
    % Run simple BFS
    path_grid = simple_bfs(map, start_grid, goal_grid);
    
    if isempty(path_grid)
        error('BFS could not find a path!');
    end
    
    % Convert to world coordinates
    path = grid2world(map, path_grid);
    
    % Moderate simplification
    waypoints = moderate_simplify(path, cfg.R_min);
end

function path = simple_bfs(map, start, goal)
    % Simple 4-connected BFS
    
    visited = false(map.GridSize);
    parent = cell(map.GridSize);
    
    queue = {start};
    visited(start(1), start(2)) = true;
    
    % 4-connected neighbors (no diagonal)
    dirs = [-1 0; 1 0; 0 -1; 0 1];
    
    found = false;
    
    while ~isempty(queue)
        current = queue{1};
        queue(1) = [];
        
        % Check if reached goal
        if current(1) == goal(1) && current(2) == goal(2)
            found = true;
            break;
        end
        
        for i = 1:size(dirs, 1)
            neighbor = current + dirs(i,:);
            
            % Check bounds
            if neighbor(1) < 1 || neighbor(1) > map.GridSize(1) || ...
               neighbor(2) < 1 || neighbor(2) > map.GridSize(2)
                continue;
            end
            
            if visited(neighbor(1), neighbor(2))
                continue;
            end
            
            % Check occupancy
            world_pos = grid2world(map, neighbor);
            if getOccupancy(map, world_pos, 'world') > 0.5
                continue;
            end
            
            visited(neighbor(1), neighbor(2)) = true;
            parent{neighbor(1), neighbor(2)} = current;
            queue{end+1} = neighbor;
        end
    end
    
    if ~found
        path = [];
        return;
    end
    
    % Reconstruct path
    path = goal;
    current = goal;
    
    while ~(current(1) == start(1) && current(2) == start(2))
        current = parent{current(1), current(2)};
        if isempty(current)
            path = [];
            return;
        end
        path = [current; path];
    end
end

function simplified = moderate_simplify(path, R_min)
    % Moderate simplification
    
    if size(path,1) <= 2
        simplified = path;
        return;
    end
    
    kept = [path(1,:)];
    
    for i = 2:(size(path,1)-1)
        v1 = path(i,:) - kept(end,:);
        v2 = path(i+1,:) - path(i,:);
        
        if norm(v1) > 1e-6 && norm(v2) > 1e-6
            v1_norm = v1 / norm(v1);
            v2_norm = v2 / norm(v2);
            angle_change = acos(max(-1, min(1, dot(v1_norm, v2_norm))));
            
            if angle_change > deg2rad(15)  % More sensitive
                kept = [kept; path(i,:)];
            end
        end
    end
    
    kept = [kept; path(end,:)];
    
    % Ensure minimum spacing
    simplified = [kept(1,:)];
    
    for i = 2:size(kept,1)
        dist = norm(kept(i,:) - simplified(end,:));
        if dist > 0.6 * R_min || i == size(kept,1)  % Increased spacing
            simplified = [simplified; kept(i,:)];
        end
    end
end
