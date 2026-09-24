% ============================================================
% INS_Mechanization - Analysis Script
% Author: Mohammed Hsiny
% Field: Electrical Engineering & Industrial Control Student
% Institution: Faculty of Sciences and Techniques of Mohammedia
% Year: 2026
% Repository: inertial-navigation-mechanization_hsiny_med
% ============================================================

function RmseWithTime(data1, data2, window_size) 
    if nargin < 3
        window_size = 100;
    end

    d1 = data1;
    d2 = data2;

    second1 = d1(:, 1);
    Vn = d1(:, 2);
    Ve = d1(:, 3);
    Vd = d1(:, 4);
    roll = d1(:, 8);
    pitch = d1(:, 9);
    yaw = d1(:, 10);

    second2 = d2(:, 1);
    dN_true = d2(:, 2);
    dE_true = d2(:, 3);
    dU_true = d2(:, 4);
    dN_our = d2(:, 5);
    dE_our = d2(:, 6);
    dU_our = d2(:, 7);

    ddN = dN_our - dN_true;
    ddE = dE_our - dE_true;
    ddU = dU_our - dU_true;

    % Moving-window RMSE
    RMSE_vn = calculate_rmse_window(second1, Vn, window_size);
    RMSE_ve = calculate_rmse_window(second1, Ve, window_size);
    RMSE_vd = calculate_rmse_window(second1, Vd, window_size);

    RMSE_n = calculate_rmse_window(second2, ddN, window_size);
    RMSE_e = calculate_rmse_window(second2, ddE, window_size);
    RMSE_u = calculate_rmse_window(second2, ddU, window_size);
    RMSE_horiz = sqrt(RMSE_n.^2 + RMSE_e.^2);
    RMSE_3d = sqrt(RMSE_n.^2 + RMSE_e.^2 + RMSE_u.^2);

    RMSE_roll = calculate_rmse_window(second1, roll, window_size);
    RMSE_pitch = calculate_rmse_window(second1, pitch, window_size);
    RMSE_yaw = calculate_rmse_window(second1, yaw, window_size);
    
    % Terminal errors summary
    mile = 1852.0;
    dN_last = ddN(end);
    dE_last = ddE(end);
    dU_last = ddU(end);

    dVn_last = Vn(end);
    dVe_last = Ve(end);
    dVd_last = Vd(end);

    droll_last = roll(end);
    dpitch_last = pitch(end);
    dyaw_last = yaw(end);

    total_time1 = max((second1(end) - second1(1)) / 3600.0, 1e-6);
    total_time2 = max((second2(end) - second2(1)) / 3600.0, 1e-6);

    fprintf("=== Terminal Drift Rates ===\n");
    fprintf("Total duration: %0.4f hours (%0.2f seconds)\n", total_time1, second1(end) - second1(1));
    position_3d_error = sqrt(dN_last^2 + dE_last^2 + dU_last^2);
    fprintf("Position Drift Rate (m/h): dN=%0.4f, dE=%0.4f, dU=%0.4f, 3D=%0.4f\n", ...
        abs(dN_last/total_time2), abs(dE_last/total_time2), abs(dU_last/total_time2), ...
        position_3d_error/total_time2);
    fprintf("Position Drift Rate (nmi/h): dN=%0.4f, dE=%0.4f, dU=%0.4f, 3D=%0.4f\n\n", ...
        abs(dN_last/total_time2)/mile, abs(dE_last/total_time2)/mile, abs(dU_last/total_time2)/mile, ...
        position_3d_error/total_time2/mile);

    if ~exist('Figures', 'dir')
        mkdir('Figures');
    end

    % ------------------------------------------------------------
    % Figure 1: 3-Subplot Position RMSE Evolution
    % ------------------------------------------------------------
    num_pts = length(RMSE_n);
    t_plot = second2(min((1:num_pts)' * window_size, length(second2)));

    h_rmse = figure('Name', 'Root Mean Square Error Evolution', 'NumberTitle', 'off');
    set(h_rmse, 'Color', 'w');

    subplot(3, 1, 1);
    plot(t_plot, RMSE_horiz, 'b-', 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Position Error (m)', 'FontSize', 11);
    title('Horizontal Position RMSE', 'FontSize', 11, 'FontWeight', 'bold');
    h_leg1 = legend('RMSE', 'Location', 'northeast');
    set(h_leg1, 'FontSize', 10);
    xlim([min(second2), max(second2)]);
    set(gca, 'FontSize', 10);

    subplot(3, 1, 2);
    plot(t_plot, RMSE_u, 'b-', 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Position Error (m)', 'FontSize', 11);
    title('Vertical Position RMSE', 'FontSize', 11, 'FontWeight', 'bold');
    h_leg2 = legend('RMSE', 'Location', 'northeast');
    set(h_leg2, 'FontSize', 10);
    xlim([min(second2), max(second2)]);
    set(gca, 'FontSize', 10);

    subplot(3, 1, 3);
    plot(t_plot, RMSE_3d, 'b-', 'LineWidth', 1.5);
    grid on; box on;
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Position Error (m)', 'FontSize', 11);
    title('3D Position RMSE', 'FontSize', 11, 'FontWeight', 'bold');
    h_leg3 = legend('RMSE', 'Location', 'northeast');
    set(h_leg3, 'FontSize', 10);
    xlim([min(second2), max(second2)]);
    set(gca, 'FontSize', 10);
    text(0.98, 0.02, 'Mohammed Hsiny - FST Mohammedia - 2026', ...
         'Units', 'normalized', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 9, ...
         'Color', [0.4 0.4 0.4]);
    print(h_rmse, '-dpng', '-r300', 'Figures/rmse_evolution.png');
end

% Moving-window RMSE calculation
function RMSE = calculate_rmse_window(time, errors, window_size)
    num_samples = length(time);
    num_windows = floor(num_samples / window_size);
    RMSE = zeros(num_windows, 1);
    
    for i = 1:num_windows
        start_idx = (i - 1) * window_size + 1;
        end_idx = min(start_idx + window_size - 1, num_samples);
        window_errors = errors(start_idx:end_idx);
        RMSE(i) = sqrt(mean(window_errors.^2));
    end
end
