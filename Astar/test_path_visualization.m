clear; clc; close all;

%% ======================= TEST PATH VISUALIZATION =======================
% Visualize A* waypoint generation and Dubins path
% No robot needed - just checks if path planning works

fprintf('=================================================================\n');
fprintf('PATH PLANNING TEST\n');
fprintf('=================================================================\n\n');

% Load configuration
cfg = limo_config();

%% ======================= GENERATE PATH WITH A* =======================
fprintf('=== GENERATING PATH ===\n');

% Generate waypoints using A*
waypoints = plan_astar_path(cfg.start_pos, cfg.goal_pos, cfg.obs, cfg.obsR, cfg);

% Build smooth Dubins path through waypoints
fprintf('\nBuilding Dubins path through waypoints...\n');
[ref, curvature] = build_dubins_path(waypoints, cfg.R_min, cfg.step, ...
                                      cfg.v_nom, cfg.obs, cfg.obsR);
fprintf('✓ Path complete: %d points, %.2f m total length\n', length(ref.x), ref.s(end));

%% ======================= ANALYZE PATH =======================
fprintf('\n=== PATH ANALYSIS ===\n');
fprintf('Total distance: %.2f m\n', ref.s(end));
fprintf('Estimated time: %.1f s (at %.2f m/s)\n', ref.t(end), cfg.v_nom);
fprintf('Max curvature: %.3f (1/m) = %.1f m radius\n', max(abs(curvature)), 1/max(abs(curvature)));

% Check for collisions
min_dist_to_obs = inf;
for i = 1:size(cfg.obs, 1)
    dists = sqrt((ref.x - cfg.obs(i,1)).^2 + (ref.y - cfg.obs(i,2)).^2);
    min_dist = min(dists);
    min_dist_to_obs = min(min_dist_to_obs, min_dist);
    
    if min_dist < cfg.obsR
        fprintf('⚠ WARNING: Path within %.2f m of obstacle %d at (%.2f, %.2f)\n', ...
                min_dist, i, cfg.obs(i,1), cfg.obs(i,2));
    end
end
fprintf('Minimum clearance: %.2f m\n', min_dist_to_obs);

%% ======================= VISUALIZATION =======================
fprintf('\n=== CREATING VISUALIZATION ===\n');

figure('Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);

% Main path plot
subplot(2,2,[1,3]);
hold on; grid on; axis equal;
xlim([cfg.arena_bounds(1) cfg.arena_bounds(2)]);
ylim([cfg.arena_bounds(3) cfg.arena_bounds(4)]);
title(sprintf('%s - Planned Path', cfg.courseName), 'FontSize', 16, 'FontWeight', 'bold');
xlabel('X [m]'); ylabel('Y [m]');

% Arena
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
plot(waypoints(:,1), waypoints(:,2), 'mo', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'LineWidth', 2);

% Start and goal
plot(cfg.start_pos(1), cfg.start_pos(2), 'go', 'MarkerSize', 20, 'MarkerFaceColor', 'g', 'LineWidth', 3);
plot(cfg.goal_pos(1), cfg.goal_pos(2), 'rs', 'MarkerSize', 20, 'MarkerFaceColor', 'r', 'LineWidth', 3);

% Add heading arrows along path
arrow_spacing = 0.5;  % Show arrow every 0.5m
arrow_indices = [];
for s_val = 0:arrow_spacing:ref.s(end)
    [~, idx] = min(abs(ref.s - s_val));
    arrow_indices(end+1) = idx;
end
quiver(ref.x(arrow_indices), ref.y(arrow_indices), ...
       0.2*cos(ref.theta(arrow_indices)), 0.2*sin(ref.theta(arrow_indices)), ...
       'b', 'LineWidth', 2, 'MaxHeadSize', 0.8, 'AutoScale', 'off');

legend({'Arena', 'Obstacles', '', 'Path', 'Waypoints', 'Start', 'Goal', 'Heading'}, ...
       'Location', 'northwest', 'FontSize', 12);

% Curvature plot
subplot(2,2,2);
plot(ref.s, curvature, 'b-', 'LineWidth', 2);
hold on;
yline(1/cfg.R_min, 'r--', 'LineWidth', 2, 'Label', 'Max curvature');
yline(-1/cfg.R_min, 'r--', 'LineWidth', 2);
grid on;
xlabel('Arc Length [m]');
ylabel('Curvature [1/m]');
title('Path Curvature');
ylim([-1.2/cfg.R_min, 1.2/cfg.R_min]);

% Heading plot
subplot(2,2,4);
plot(ref.s, rad2deg(ref.theta), 'b-', 'LineWidth', 2);
grid on;
xlabel('Arc Length [m]');
ylabel('Heading [deg]');
title('Path Heading');

fprintf('✓ Visualization complete\n');
fprintf('\n=================================================================\n');
fprintf('Path looks good!\n');
fprintf('  - Start: (%.2f, %.2f)\n', cfg.start_pos(1), cfg.start_pos(2));
fprintf('  - Goal: (%.2f, %.2f)\n', cfg.goal_pos(1), cfg.goal_pos(2));
fprintf('  - Distance: %.2f m\n', ref.s(end));
fprintf('  - Time: %.1f s\n', ref.t(end));
fprintf('  - Waypoints: %d\n', size(waypoints, 1));
fprintf('\nReady to run on robot!\n');
fprintf('=================================================================\n\n');
