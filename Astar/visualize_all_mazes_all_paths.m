clear; clc; close all;

%% ======================= MULTI-MAZE + MULTI-ALGORITHM VISUALIZER =======================
% Visualize all maze courses with A*, DFS, and BFS paths

fprintf('=================================================================\n');
fprintf('MULTI-MAZE PATH VISUALIZER (A*, DFS, BFS)\n');
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
    
    %% Generate paths with all 3 algorithms
    paths_found = struct();
    
    % Try A*
    try
        fprintf('Planning with A*...\n');
        waypoints_astar = plan_astar_path(cfg.start_pos, cfg.goal_pos, cfg.obs, cfg.obsR, cfg);
        [ref_astar, ~] = build_dubins_path(waypoints_astar, cfg.R_min, cfg.step, ...
                                            cfg.v_nom, cfg.obs, cfg.obsR);
        fprintf('  ✓ A*:  %d wps, %.2f m\n', size(waypoints_astar,1), ref_astar.s(end));
        paths_found.astar = true;
    catch ME
        fprintf('  ✗ A* failed: %s\n', ME.message);
        paths_found.astar = false;
    end
    
    % Try DFS
    try
        fprintf('Planning with DFS...\n');
        waypoints_dfs = plan_dfs_path(cfg.start_pos, cfg.goal_pos, cfg.obs, cfg.obsR, cfg);
        [ref_dfs, ~] = build_dubins_path(waypoints_dfs, cfg.R_min, cfg.step, ...
                                          cfg.v_nom, cfg.obs, cfg.obsR);
        fprintf('  ✓ DFS: %d wps, %.2f m\n', size(waypoints_dfs,1), ref_dfs.s(end));
        paths_found.dfs = true;
    catch ME
        fprintf('  ✗ DFS failed: %s\n', ME.message);
        paths_found.dfs = false;
    end
    
    % Try BFS
    try
        fprintf('Planning with BFS...\n');
        waypoints_bfs = plan_bfs_path(cfg.start_pos, cfg.goal_pos, cfg.obs, cfg.obsR, cfg);
        [ref_bfs, ~] = build_dubins_path(waypoints_bfs, cfg.R_min, cfg.step, ...
                                          cfg.v_nom, cfg.obs, cfg.obsR);
        fprintf('  ✓ BFS: %d wps, %.2f m\n', size(waypoints_bfs,1), ref_bfs.s(end));
        paths_found.bfs = true;
    catch ME
        fprintf('  ✗ BFS failed: %s\n', ME.message);
        paths_found.bfs = false;
    end
    
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
    scatter(cfg.obs(:,1), cfg.obs(:,2), 100, 'k', 'filled');
    for i = 1:size(cfg.obs, 1)
        viscircles([cfg.obs(i,1), cfg.obs(i,2)], cfg.obsR, ...
                   'Color', [0.7 0.7 0.7], 'LineStyle', '--', 'LineWidth', 1);
    end
    
    % Paths - different colors and styles (only if found)
    legend_entries = {'Arena', 'Obstacles', ''};
    if paths_found.astar
        plot(ref_astar.x, ref_astar.y, 'r-', 'LineWidth', 3);
        legend_entries{end+1} = 'A*';
    end
    if paths_found.dfs
        plot(ref_dfs.x, ref_dfs.y, 'b--', 'LineWidth', 2.5);
        legend_entries{end+1} = 'DFS';
    end
    if paths_found.bfs
        plot(ref_bfs.x, ref_bfs.y, 'g:', 'LineWidth', 2.5);
        legend_entries{end+1} = 'BFS';
    end
    
    % Start and goal
    plot(cfg.start_pos(1), cfg.start_pos(2), 'ko', 'MarkerSize', 18, 'MarkerFaceColor', 'g', 'LineWidth', 3);
    plot(cfg.goal_pos(1), cfg.goal_pos(2), 'ks', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'LineWidth', 3);
    legend_entries{end+1} = 'Start';
    legend_entries{end+1} = 'Goal';
    
    % Info text
    info_str = '';
    if paths_found.astar
        info_str = [info_str sprintf('A*:  %.1fm (%d wps)\n', ref_astar.s(end), size(waypoints_astar,1))];
    else
        info_str = [info_str sprintf('A*:  FAILED\n')];
    end
    if paths_found.dfs
        info_str = [info_str sprintf('DFS: %.1fm (%d wps)\n', ref_dfs.s(end), size(waypoints_dfs,1))];
    else
        info_str = [info_str sprintf('DFS: FAILED\n')];
    end
    if paths_found.bfs
        info_str = [info_str sprintf('BFS: %.1fm (%d wps)', ref_bfs.s(end), size(waypoints_bfs,1))];
    else
        info_str = [info_str sprintf('BFS: FAILED')];
    end
    text(0.02, 0.98, info_str, 'Units', 'normalized', ...
         'VerticalAlignment', 'top', 'FontSize', 10, ...
         'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    if maze_idx == 1
        legend(legend_entries, 'Location', 'southwest', 'FontSize', 9);
    end
end

fprintf('\n=================================================================\n');
fprintf('All paths generated successfully!\n');
fprintf('=================================================================\n\n');
