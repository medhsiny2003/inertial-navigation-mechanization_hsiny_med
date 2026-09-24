% ============================================================
% INS_Mechanization - Analysis Script
% Author: Mohammed Hsiny
% Field: Electrical Engineering & Industrial Control Student
% Institution: Faculty of Sciences and Techniques of Mohammedia
% Year: 2026
% Repository: inertial-navigation-mechanization_hsiny_med
% ============================================================

function dEnuFig(data)
    % Extract numeric matrix safely
    d = data;

    second = d(:, 1);
    dN_true = d(:, 2);
    dE_true = d(:, 3);
    dU_true = d(:, 4);
    dN_our = d(:, 5);
    dE_our = d(:, 6);
    dU_our = d(:, 7);

    ddN = dN_our - dN_true;
    ddE = dE_our - dE_true;
    ddU = dU_our - dU_true;

    fprintf("=== ENU Trajectory Errors ===\n");
    fprintf("  dN_mean = %0.10f m, dE_mean = %0.10f m, dU_mean = %0.10f m\n", ...
        mean(ddN), mean(ddE), mean(ddU));
    fprintf("  dN_RMSE = %0.10f m, dE_RMSE = %0.10f m, dU_RMSE = %0.10f m\n\n", ...
        calculate_rmse(ddN), calculate_rmse(ddE), calculate_rmse(ddU));

    if ~exist('Figures', 'dir')
        mkdir('Figures');
    end

    % ------------------------------------------------------------
    % Figure 1: 3D Trajectory Comparison
    % ------------------------------------------------------------
    h_3d = figure('Name', '3D Trajectory: Estimated vs Ground Truth', 'NumberTitle', 'off');
    set(h_3d, 'Color', 'w');
    plot3(dE_our, dN_our, dU_our, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
    hold on;
    plot3(dE_true, dN_true, dU_true, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
    grid on; box on;
    xlabel('East (m)', 'FontSize', 11);
    ylabel('North (m)', 'FontSize', 11);
    zlabel('Up (m)', 'FontSize', 11);
    title('3D Trajectory: Estimated vs Ground Truth', 'FontSize', 12, 'FontWeight', 'bold');
    h_leg1 = legend('Estimated', 'Ground Truth', 'Location', 'best');
    set(h_leg1, 'FontSize', 10);
    set(gca, 'FontSize', 11);
    view(45, 30);
    text(0.98, 0.02, 'Mohammed Hsiny - FST Mohammedia - 2026', ...
         'Units', 'normalized', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 9, ...
         'Color', [0.4 0.4 0.4]);
    hold off;
    print(h_3d, '-dpng', '-r300', 'Figures/trajectory_3d.png');

    % ------------------------------------------------------------
    % Figure 2: 2D Horizontal Trajectory (North-East Plane)
    % ------------------------------------------------------------
    h_2d = figure('Name', 'Horizontal Trajectory (North-East Plane)', 'NumberTitle', 'off');
    set(h_2d, 'Color', 'w');
    plot(dE_our, dN_our, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
    hold on;
    plot(dE_true, dN_true, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
    grid on; box on;
    xlabel('East (m)', 'FontSize', 11);
    ylabel('North (m)', 'FontSize', 11);
    title('Horizontal Trajectory (North-East Plane)', 'FontSize', 12, 'FontWeight', 'bold');
    h_leg2 = legend('Estimated', 'Ground Truth', 'Location', 'best');
    set(h_leg2, 'FontSize', 10);
    set(gca, 'FontSize', 11);
    axis equal;
    text(0.98, 0.02, 'Mohammed Hsiny - FST Mohammedia - 2026', ...
         'Units', 'normalized', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 9, ...
         'Color', [0.4 0.4 0.4]);
    hold off;
    print(h_2d, '-dpng', '-r300', 'Figures/trajectory_2d.png');

    % ------------------------------------------------------------
    % Figure 3: Position Errors in Local ENU Frame
    % ------------------------------------------------------------
    h_enu = figure('Name', 'Position Errors in Local ENU Frame', 'NumberTitle', 'off');
    set(h_enu, 'Color', 'w');

    subplot(3, 1, 1);
    plot(second, ddE, 'k-', 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Position Error (m)', 'FontSize', 11);
    title('East Position Error (dE)', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]);
    set(gca, 'FontSize', 10);

    subplot(3, 1, 2);
    plot(second, ddN, 'k-', 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Position Error (m)', 'FontSize', 11);
    title('North Position Error (dN)', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]);
    set(gca, 'FontSize', 10);

    subplot(3, 1, 3);
    plot(second, ddU, 'k-', 'LineWidth', 1.5);
    grid on; box on;
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Position Error (m)', 'FontSize', 11);
    title('Up Position Error (dU)', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]);
    set(gca, 'FontSize', 10);
    text(0.98, 0.02, 'Mohammed Hsiny - FST Mohammedia - 2026', ...
         'Units', 'normalized', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 9, ...
         'Color', [0.4 0.4 0.4]);
    print(h_enu, '-dpng', '-r300', 'Figures/denu_errors.png');
end

function rmse_value = calculate_rmse(errors)
    rmse_value = sqrt(mean(errors.^2));
end