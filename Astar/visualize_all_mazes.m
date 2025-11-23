clear; clc; close all;

%% ======================= MULTI-MAZE PATH VISUALIZER =======================
% Visualize all maze courses with A* + Dubins paths

fprintf('=================================================================\n');
fprintf('MULTI-MAZE PATH VISUALIZER\n');
fprintf('=================================================================\n\n');

% Load configuration
cfg = limo_config();

%% Create figure with 3 subplots
figure('Units', 'normalized', 'Position', [0.05 0.05 0.9 0.9]);

for maze_idx = 1:3
    % Select maze
    cfg.maze_select = maze_idx;
    cfg.obs = cfg.mazes(maze_idx).obs;
    cfg.start_pos = cfg.mazes(maze_idx).start_pos;
    cfg.goal_pos = cfg.mazes(maze_idx).goal_pos;
    cfg.arena_bounds = cfg.mazes(maze_idx).arena_bounds;
    cfg.courseName = cfg.mazes(maze_idx).name;
    
    fprintf('\n=== MAZE %d: %s ===\n', maze_idx, cfg.courseName);
    
    %% Generate path
    fprintf('Planning path...\n');
    
    % A* waypoints
    waypoints = plan_astar_path(cfg.start_pos, cfg.goal_pos, cfg.obs, cfg.obsR, cfg);
    
    % Dubins path
    [ref, curvature] = build_dubins_path(waypoints, cfg.R_min, cfg.step, ...
                                          cfg.v_nom, cfg.obs, cfg.obsR);
    
    fprintf('✓ Path: %d points, %.2f m, %.1f s\n', ...
            length(ref.x), ref.s(end), ref.t(end));
    
    % Check clearance
    min_dist_to_obs = inf;
    for i = 1:size(cfg.obs, 1)
        dists = sqrt((ref.x - cfg.obs(i,1)).^2 + (ref.y - cfg.obs(i,2)).^2);
        min_dist = min(dists);
        min_dist_to_obs = min(min_dist_to_obs, min_dist);
    end
    fprintf('  Clearance: %.2f m\n', min_dist_to_obs);
    fprintf('  Max curvature: %.2f (1/m)\n', max(abs(curvature)));
    
    %% Plot
    subplot(1, 3, maze_idx);
    hold on; grid on; axis equal;
    xlim([cfg.arena_bounds(1) cfg.arena_bounds(2)]);
    ylim([cfg.arena_bounds(3) cfg.arena_bounds(4)]);
    title(sprintf('Maze %d: %s', maze_idx, cfg.courseName), ...
          'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X [m]'); ylabel('Y [m]');
    
    % Arena boundary
    plot([cfg.arena_bounds(1) cfg.arena_bounds(2) cfg.arena_bounds(2) cfg.arena_bounds(1) cfg.arena_bounds(1)], ...
         [cfg.arena_bounds(3) cfg.arena_bounds(3) cfg.arena_bounds(4) cfg.arena_bounds(4) cfg.arena_bounds(3)], ...
         'k', 'LineWidth', 3);
    
    % Obstacles
    scatter(cfg.obs(:,1), cfg.obs(:,2), 150, 'k', 'filled');
    for i = 1:size(cfg.obs, 1)
        viscircles([cfg.obs(i,1), cfg.obs(i,2)], cfg.obsR, ...
                   'Color', [0.8 0 0], 'LineStyle', '--', 'LineWidth', 2);
    end
    
    % Path
    plot(ref.x, ref.y, 'r-', 'LineWidth', 3);
    
    % Waypoints
    plot(waypoints(:,1), waypoints(:,2), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'LineWidth', 2);
    
    % Start and goal
    plot(cfg.start_pos(1), cfg.start_pos(2), 'go', 'MarkerSize', 20, 'MarkerFaceColor', 'g', 'LineWidth', 3);
    plot(cfg.goal_pos(1), cfg.goal_pos(2), 'rs', 'MarkerSize', 20, 'MarkerFaceColor', 'r', 'LineWidth', 3);
    
    % Heading arrows
    arrow_spacing = 0.5;
    arrow_indices = [];
    for s_val = 0:arrow_spacing:ref.s(end)
        [~, idx] = min(abs(ref.s - s_val));
        arrow_indices(end+1) = idx;
    end
    quiver(ref.x(arrow_indices), ref.y(arrow_indices), ...
           0.2*cos(ref.theta(arrow_indices)), 0.2*sin(ref.theta(arrow_indices)), ...
           'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.8, 'AutoScale', 'off');
    
    % Info text
    info_str = sprintf('Distance: %.1fm\nTime: %.1fs\nWaypoints: %d', ...
                       ref.s(end), ref.t(end), size(waypoints, 1));
    text(0.02, 0.98, info_str, 'Units', 'normalized', ...
         'VerticalAlignment', 'top', 'FontSize', 10, ...
         'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    if maze_idx == 1
        legend({'Arena', 'Obstacles', '', 'Path', 'Waypoints', 'Start', 'Goal', 'Heading'}, ...
               'Location', 'southwest', 'FontSize', 9);
    end
end

fprintf('\n=================================================================\n');
fprintf('All maze paths generated successfully!\n');
fprintf('=================================================================\n\n');
