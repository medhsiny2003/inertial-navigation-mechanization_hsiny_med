% ============================================================
% INS_Mechanization - Analysis Script
% Author: Mohammed Hsiny
% Field: Electrical Engineering & Industrial Control Student
% Institution: Faculty of Sciences and Techniques of Mohammedia
% Year: 2026
% Repository: inertial-navigation-mechanization_hsiny_med
% ============================================================

function DiffFig(data, mode)
    if nargin < 2
        mode = 1;
    end

    d = data;

    second = d(:, 1);
    Vn = d(:, 2);
    Ve = d(:, 3);
    Vd = d(:, 4);
    B = d(:, 5);
    L = d(:, 6);
    H = d(:, 7);
    roll = d(:, 8);
    pitch = d(:, 9);
    yaw = d(:, 10);

    fprintf("=== Position Errors ===\n");
    fprintf("  dLat_mean = %0.10f deg, dLon_mean = %0.10f deg, dH_mean = %0.10f m\n", ...
        mean(B), mean(L), mean(H));
    fprintf("  dLat_RMSE = %0.10f deg, dLon_RMSE = %0.10f deg, dH_RMSE = %0.10f m\n\n", ...
        calculate_rmse(B), calculate_rmse(L), calculate_rmse(H));

    fprintf("=== Velocity Errors ===\n");
    fprintf("  dVn_mean = %0.10f m/s, dVe_mean = %0.10f m/s, dVd_mean = %0.10f m/s\n", ...
        mean(Vn), mean(Ve), mean(Vd));
    fprintf("  dVn_RMSE = %0.10f m/s, dVe_RMSE = %0.10f m/s, dVd_RMSE = %0.10f m/s\n\n", ...
        calculate_rmse(Vn), calculate_rmse(Ve), calculate_rmse(Vd));

    fprintf("=== Attitude Errors ===\n");
    fprintf("  dRoll_mean = %0.10f deg, dPitch_mean = %0.10f deg, dYaw_mean = %0.10f deg\n", ...
        mean(roll), mean(pitch), mean(yaw));
    fprintf("  dRoll_RMSE = %0.10f deg, dPitch_RMSE = %0.10f deg, dYaw_RMSE = %0.10f deg\n\n", ...
        calculate_rmse(roll), calculate_rmse(pitch), calculate_rmse(yaw));

    if ~exist('Figures', 'dir')
        mkdir('Figures');
    end

    % ------------------------------------------------------------
    % Figure 1: Velocity Errors
    % ------------------------------------------------------------
    h_vel = figure('Name', 'Velocity Errors', 'NumberTitle', 'off');
    set(h_vel, 'Color', 'w');

    subplot(3, 1, 1);
    plot(second, Vn, 'k-', 'LineWidth', 1.5);
    grid on; box on; ylabel('dVn (m/s)', 'FontSize', 11);
    title('North Velocity Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 10);

    subplot(3, 1, 2);
    plot(second, Ve, 'k-', 'LineWidth', 1.5);
    grid on; box on; ylabel('dVe (m/s)', 'FontSize', 11);
    title('East Velocity Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 10);

    subplot(3, 1, 3);
    plot(second, Vd, 'k-', 'LineWidth', 1.5);
    grid on; box on; xlabel('Time (s)', 'FontSize', 11); ylabel('dVd (m/s)', 'FontSize', 11);
    title('Down Velocity Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 10);
    text(0.98, 0.02, 'Mohammed Hsiny - FST Mohammedia - 2026', ...
         'Units', 'normalized', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 9, ...
         'Color', [0.4 0.4 0.4]);
    print(h_vel, '-dpng', '-r300', 'Figures/velocity_errors.png');

    % ------------------------------------------------------------
    % Figure 2: Position Errors (Latitude, Longitude, Height)
    % ------------------------------------------------------------
    h_pos = figure('Name', 'Position Errors', 'NumberTitle', 'off');
    set(h_pos, 'Color', 'w');

    subplot(3, 1, 1);
    plot(second, B, 'k-', 'LineWidth', 1.5);
    grid on; box on; ylabel('dLat (deg)', 'FontSize', 11);
    title('Latitude Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 10);

    subplot(3, 1, 2);
    plot(second, L, 'k-', 'LineWidth', 1.5);
    grid on; box on; ylabel('dLon (deg)', 'FontSize', 11);
    title('Longitude Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 10);

    subplot(3, 1, 3);
    plot(second, H, 'k-', 'LineWidth', 1.5);
    grid on; box on; xlabel('Time (s)', 'FontSize', 11); ylabel('dH (m)', 'FontSize', 11);
    title('Height Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 10);
    text(0.98, 0.02, 'Mohammed Hsiny - FST Mohammedia - 2026', ...
         'Units', 'normalized', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 9, ...
         'Color', [0.4 0.4 0.4]);
    print(h_pos, '-dpng', '-r300', 'Figures/position_errors.png');

    % ------------------------------------------------------------
    % Figure 3: Full 9-Subplot Estimation Errors
    % ------------------------------------------------------------
    h_all = figure('Name', 'Position, Velocity, and Attitude Errors', 'NumberTitle', 'off');
    set(h_all, 'Color', 'w');

    subplot(3, 3, 1);
    plot(second, Vn, 'k-', 'LineWidth', 1.5);
    grid on; box on; ylabel('dVn (m/s)', 'FontSize', 10);
    title('North Velocity Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 9);

    subplot(3, 3, 2);
    plot(second, Ve, 'k-', 'LineWidth', 1.5);
    grid on; box on; ylabel('dVe (m/s)', 'FontSize', 10);
    title('East Velocity Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 9);

    subplot(3, 3, 3);
    plot(second, Vd, 'k-', 'LineWidth', 1.5);
    grid on; box on; ylabel('dVd (m/s)', 'FontSize', 10);
    title('Down Velocity Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 9);

    subplot(3, 3, 4);
    plot(second, B, 'k-', 'LineWidth', 1.5);
    grid on; box on; ylabel('dLat (deg)', 'FontSize', 10);
    title('Latitude Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 9);

    subplot(3, 3, 5);
    plot(second, L, 'k-', 'LineWidth', 1.5);
    grid on; box on; ylabel('dLon (deg)', 'FontSize', 10);
    title('Longitude Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 9);

    subplot(3, 3, 6);
    plot(second, H, 'k-', 'LineWidth', 1.5);
    grid on; box on; ylabel('dH (m)', 'FontSize', 10);
    title('Height Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 9);

    subplot(3, 3, 7);
    plot(second, roll, 'k-', 'LineWidth', 1.5);
    grid on; box on; xlabel('Time (s)', 'FontSize', 10); ylabel('dRoll (deg)', 'FontSize', 10);
    title('Roll Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 9);

    subplot(3, 3, 8);
    plot(second, pitch, 'k-', 'LineWidth', 1.5);
    grid on; box on; xlabel('Time (s)', 'FontSize', 10); ylabel('dPitch (deg)', 'FontSize', 10);
    title('Pitch Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 9);

    subplot(3, 3, 9);
    plot(second, yaw, 'k-', 'LineWidth', 1.5);
    grid on; box on; xlabel('Time (s)', 'FontSize', 10); ylabel('dYaw (deg)', 'FontSize', 10);
    title('Yaw Error', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([min(second), max(second)]); set(gca, 'FontSize', 9);
    text(0.98, 0.02, 'Mohammed Hsiny - FST Mohammedia - 2026', ...
         'Units', 'normalized', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 8, ...
         'Color', [0.4 0.4 0.4]);
end

function rmse_value = calculate_rmse(errors)
    rmse_value = sqrt(mean(errors.^2));
end
