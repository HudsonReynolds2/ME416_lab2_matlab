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
    
    % Clear start/goal regions
    clearance = res * 1.5;
    setOccupancy(map, start_pos, 0, 'world');
    setOccupancy(map, goal_pos, 0, 'world');
    
    [X_local, Y_local] = meshgrid(start_pos(1)-clearance:res:start_pos(1)+clearance, ...
                                   start_pos(2)-clearance:res:start_pos(2)+clearance);
    start_region = [X_local(:), Y_local(:)];
    for i = 1:size(start_region, 1)
        if start_region(i,1) >= xmin && start_region(i,1) <= xmax && ...
           start_region(i,2) >= ymin && start_region(i,2) <= ymax
            setOccupancy(map, start_region(i,:), 0, 'world');
        end
    end
    
    [X_local, Y_local] = meshgrid(goal_pos(1)-clearance:res:goal_pos(1)+clearance, ...
                                   goal_pos(2)-clearance:res:goal_pos(2)+clearance);
    goal_region = [X_local(:), Y_local(:)];
    for i = 1:size(goal_region, 1)
        if goal_region(i,1) >= xmin && goal_region(i,1) <= xmax && ...
           goal_region(i,2) >= ymin && goal_region(i,2) <= ymax
            setOccupancy(map, goal_region(i,:), 0, 'world');
        end
    end
    
    % Convert to grid coordinates
    start_grid = world2grid(map, start_pos);
    goal_grid = world2grid(map, goal_pos);
    
    % Run BFS
    path_grid = bfs_search(map, start_grid, goal_grid);
    
    if isempty(path_grid)
        error('BFS could not find a path!');
    end
    
    % Convert back to world coordinates
    path = grid2world(map, path_grid);
    
    % Simplify
    waypoints = simplify_path(path, res);
end

function path = bfs_search(map, start, goal)
    % BFS implementation with iteration limit
    visited = containers.Map('KeyType', 'int64', 'ValueType', 'logical');
    parent = containers.Map('KeyType', 'int64', 'ValueType', 'any');
    
    % Convert grid coords to single integer key
    gridSize = map.GridSize;
    toKey = @(pos) int64(pos(1) * 10000 + pos(2));
    
    queue = {start};
    visited(toKey(start)) = true;
    
    % 4-connected neighbors (faster than 8-connected)
    dirs = [-1 0; 1 0; 0 -1; 0 1];
    
    found = false;
    max_iters = 20000;  % Increased limit for complex mazes
    iters = 0;
    
    while ~isempty(queue) && iters < max_iters
        iters = iters + 1;
        current = queue{1};
        queue(1) = [];
        
        % Check if we reached goal (exact match)
        if current(1) == goal(1) && current(2) == goal(2)
            found = true;
            break;
        end
        
        for i = 1:size(dirs, 1)
            neighbor = current + dirs(i,:);
            key = toKey(neighbor);
            
            if visited.isKey(key)
                continue;
            end
            
            % Check bounds
            if neighbor(1) < 1 || neighbor(1) > gridSize(1) || ...
               neighbor(2) < 1 || neighbor(2) > gridSize(2)
                continue;
            end
            
            % Check occupancy
            world_pos = grid2world(map, neighbor);
            if getOccupancy(map, world_pos, 'world') > 0.5
                continue;
            end
            
            visited(key) = true;
            parent(key) = current;
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
    max_path_len = 1000;  % Prevent infinite loops
    path_len = 1;
    
    while ~(current(1) == start(1) && current(2) == start(2)) && path_len < max_path_len
        key = toKey(current);
        if ~parent.isKey(key)
            % Path reconstruction failed
            path = [];
            return;
        end
        current = parent(key);
        path = [current; path];
        path_len = path_len + 1;
    end
end

function waypoints = simplify_path(path, res)
    % Remove collinear points
    simplified = [path(1,:)];
    
    for i = 2:(size(path,1)-1)
        v1 = path(i,:) - simplified(end,:);
        v2 = path(i+1,:) - path(i,:);
        
        if norm(v1) < 1e-6 || norm(v2) < 1e-6
            continue;
        end
        
        v1 = v1 / norm(v1);
        v2 = v2 / norm(v2);
        
        if abs(acos(max(-1, min(1, dot(v1, v2))))) > deg2rad(10)
            simplified = [simplified; path(i,:)];
        end
    end
    
    waypoints = [simplified; path(end,:)];
end
