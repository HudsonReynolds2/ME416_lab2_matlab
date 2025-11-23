function [ref, curvature] = build_dubins_path(wps_2d, R_min, step, v_nom, obs, obsR)
    %% BUILD_DUBINS_PATH - Generate collision-free Dubins path through waypoints
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
    %   ref         - Structure with fields:
    %                 .x, .y, .theta (path coordinates)
    %                 .s (arc length)
    %                 .t (time along path)
    %   curvature   - Curvature at each point
    
    %% Check inputs
    if nargin < 5
        obs = [];
        obsR = 0;
    end
    
    %% Helper functions
    dirAngle = @(p1,p2) atan2(p2(2)-p1(2), p2(1)-p1(1));
    
    function pts = dubinsInterp(segObj, step)
        L = segObj.Length;
        s = 0:step:L;
        pts = interpolate(segObj, s);
    end
    
    function tf = collides(path, obs, R)
        tf = false;
        if isempty(obs), return; end
        for i = 1:size(obs,1)
            if any(vecnorm(path(:,1:2)-obs(i,:),2,2) < R)
                tf = true;
                return;
            end
        end
    end
    
    %% Build waypoints with headings
    % Important: Heading at each waypoint points toward NEXT waypoint
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
    
    %% Connect waypoints with Dubins curves
    dubConn = dubinsConnection;
    dubConn.MinTurningRadius = R_min;
    fullPath = [];
    
    for k = 1:(size(wps,1)-1)
        p1 = wps(k,:);
        p2 = wps(k+1,:);
        
        % Get all possible Dubins paths (LSL, RSR, LSR, RSL, RLR, LRL)
        [segList, costs] = connect(dubConn, p1, p2, "PathSegments", "all");
        
        % Find shortest collision-free path
        found_safe = false;
        [sorted_costs, sort_idx] = sort(costs);
        
        for i = 1:length(segList)
            idx = sort_idx(i);
            pts = dubinsInterp(segList{idx}, step);
            
            % Check collision with actual obstacle radius
            if ~collides(pts, obs, obsR)
                fullPath = [fullPath; pts];
                found_safe = true;
                break;
            end
        end
        
        if ~found_safe
            error('No collision-free Dubins path from WP %d to %d. A* waypoints too close to obstacles.', k, k+1);
        end
    end
    
    %% Extract coordinates
    x = fullPath(:,1);
    y = fullPath(:,2);
    theta = fullPath(:,3);
    
    %% Compute arc length
    dx = diff(x);
    dy = diff(y);
    ds = sqrt(dx.^2 + dy.^2);
    s = [0; cumsum(ds)];
    
    %% Compute time (assuming constant velocity)
    t = s / v_nom;
    
    %% Compute curvature (κ = dθ/ds)
    dtheta = diff(theta);
    dtheta = wrapToPi(dtheta);  % Handle angle wrapping
    kappa = zeros(size(theta));
    kappa(2:end) = dtheta ./ max(ds, 1e-6);
    kappa(1) = kappa(2);  % Copy first valid curvature to first point
    
    % Smooth curvature to remove numerical spikes
    if length(kappa) > 5
        kappa = movmean(kappa, 5);
    end
    
    %% Package output
    ref.x = x;
    ref.y = y;
    ref.theta = theta;
    ref.s = s;
    ref.t = t;
    curvature = kappa;
    
end
