function waypoints = plan_dfs_path(start_pos, goal_pos, obstacles, obsR, cfg)
    %% PLAN_DFS_PATH - Generate waypoints using Depth-First Search
    % Uses guided DFS with goal-biased ordering to avoid excessive wandering
    
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
    
    % Clear regions around start and goal
    clearance_radius = res * 2;
    
    % Clear start region
    for dx = -clearance_radius:res:clearance_radius
        for dy = -clearance_radius:res:clearance_radius
            pos = start_pos + [dx, dy];
            if pos(1) >= xmin && pos(1) <= xmax && pos(2) >= ymin && pos(2) <= ymax
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
                min_dist = min(sqrt((obstacles(:,1) - pos(1)).^2 + (obstacles(:,2) - pos(2)).^2));
                if min_dist > obsR
                    setOccupancy(map, pos, 0, 'world');
                end
            end
        end
    end
    
    % Convert to grid coordinates
    start_grid = world2grid(map, start_pos);
    goal_grid = world2grid(map, goal_pos);
    
    % Run guided DFS
    path_grid = dfs_search_guided(map, start_grid, goal_grid);
    
    if isempty(path_grid)
        error('DFS could not find a path!');
    end
    
    % Convert back to world coordinates
    path = grid2world(map, path_grid);
    
    % Simplify
    waypoints = simplify_path_smart(path, obstacles, obsR, cfg.R_min);
end

function path = dfs_search_guided(map, start, goal)
    % Guided DFS with goal-biased neighbor ordering
    
    visited = containers.Map('KeyType', 'int64', 'ValueType', 'logical');
    parent = containers.Map('KeyType', 'int64', 'ValueType', 'any');
    
    % Convert grid coords to single integer key
    gridSize = map.GridSize;
    toKey = @(pos) int64(pos(1) * 100000 + pos(2));
    
    stack = {start};
    visited(toKey(start)) = true;
    
    % 8-connected neighbors
    dirs = [-1 0; 1 0; 0 -1; 0 1;   % Cardinal
            -1 -1; -1 1; 1 -1; 1 1]; % Diagonal
    
    found = false;
    max_iters = 20000;
    iters = 0;
    
    while ~isempty(stack) && iters < max_iters
        iters = iters + 1;
        current = stack{end};
        stack(end) = [];
        
        % Check if we reached goal
        if abs(current(1) - goal(1)) <= 1 && abs(current(2) - goal(2)) <= 1
            found = true;
            break;
        end
        
        % Collect valid neighbors
        valid_neighbors = [];
        
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
            
            % For diagonal moves, check corners
            if i > 4
                cardinal1 = current + dirs(mod(i-5, 4) + 1, :);
                cardinal2 = current + dirs(mod(i-4, 4) + 1, :);
                
                if cardinal1(1) >= 1 && cardinal1(1) <= gridSize(1) && ...
                   cardinal1(2) >= 1 && cardinal1(2) <= gridSize(2)
                    world_pos1 = grid2world(map, cardinal1);
                    if getOccupancy(map, world_pos1, 'world') > 0.5
                        continue;
                    end
                end
                
                if cardinal2(1) >= 1 && cardinal2(1) <= gridSize(1) && ...
                   cardinal2(2) >= 1 && cardinal2(2) <= gridSize(2)
                    world_pos2 = grid2world(map, cardinal2);
                    if getOccupancy(map, world_pos2, 'world') > 0.5
                        continue;
                    end
                end
            end
            
            valid_neighbors = [valid_neighbors; neighbor];
        end
        
        % Sort neighbors by distance to goal (prefer closer)
        if ~isempty(valid_neighbors)
            dists = sum((valid_neighbors - repmat(goal, size(valid_neighbors,1), 1)).^2, 2);
            [~, idx] = sort(dists, 'descend');  % Reverse order for stack (last is popped first)
            
            for i = 1:size(valid_neighbors,1)
                neighbor = valid_neighbors(idx(i),:);
                key = toKey(neighbor);
                visited(key) = true;
                parent(key) = current;
                stack{end+1} = neighbor;
            end
        end
    end
    
    if ~found
        path = [];
        return;
    end
    
    % Reconstruct path
    path = goal;
    current = goal;
    max_path_len = 2000;
    path_len = 1;
    
    while ~(abs(current(1) - start(1)) <= 1 && abs(current(2) - start(2)) <= 1) && path_len < max_path_len
        key = toKey(current);
        if ~parent.isKey(key)
            path = [];
            return;
        end
        current = parent(key);
        path = [current; path];
        path_len = path_len + 1;
    end
end

function simplified = simplify_path_smart(path, obstacles, obsR, R_min)
    % Smart path simplification
    
    if size(path,1) <= 2
        simplified = path;
        return;
    end
    
    simplified = [path(1,:)];
    
    i = 1;
    while i < size(path,1)
        best_skip = i + 1;
        
        for j = (i+2):min(i+20, size(path,1))
            if can_connect(path(i,:), path(j,:), obstacles, obsR)
                best_skip = j;
            else
                break;
            end
        end
        
        simplified = [simplified; path(best_skip,:)];
        i = best_skip;
    end
    
    if ~isequal(simplified(end,:), path(end,:))
        simplified = [simplified; path(end,:)];
    end
    
    % Remove collinear points
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
        
        if angle_change > deg2rad(15)
            final = [final; simplified(i,:)];
        end
    end
    
    final = [final; simplified(end,:)];
    
    % Ensure minimum spacing
    spaced = [final(1,:)];
    for i = 2:size(final,1)
        if norm(final(i,:) - spaced(end,:)) > 0.5 * R_min || i == size(final,1)
            spaced = [spaced; final(i,:)];
        end
    end
    
    simplified = spaced;
end

function can_connect = can_connect(p1, p2, obstacles, obsR)
    n_checks = ceil(norm(p2 - p1) / 0.1);
    
    for alpha = linspace(0, 1, n_checks)
        point = p1 + alpha * (p2 - p1);
        dists = sqrt((obstacles(:,1) - point(1)).^2 + (obstacles(:,2) - point(2)).^2);
        if min(dists) <= obsR + 0.1
            can_connect = false;
            return;
        end
    end
    
    can_connect = true;
end
