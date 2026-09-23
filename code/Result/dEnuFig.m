% ============================================================
% INS_Mechanization - Analysis Scripts
% Author: Mohammed Hsiny
% Based on the original project by GYH-WHU
% Year: 2025
% ============================================================

function dEnuFig(data)
    % Convert table columns to numeric arrays
    second = table2array(data(:,1));
    dN_true = table2array(data(:,2));
    dE_true = table2array(data(:,3));
    dU_true = table2array(data(:,4));

    dN_our = table2array(data(:,5));
    dE_our = table2array(data(:,6));
    dU_our = table2array(data(:,7));
    
    ddN = dN_our - dN_true;
    ddE = dE_our - dE_true;
    ddU = dU_our - dU_true;

    fprintf("  dN_mean = %0.10f m, dE_mean = %0.10f m, dU_mean = %0.10f m\n", ...
        mean(ddN), mean(ddE), mean(ddU));
    fprintf("  dN_RMSE = %0.10f m, dE_RMSE = %0.10f m, dU_RMSE = %0.10f m\n", ...
        calculate_rmse(ddN), calculate_rmse(ddE), calculate_rmse(ddU));

    % 1. Plot 2D Trajectory (North vs East)
    figure('Name', '2D Trajectory', 'NumberTitle', 'off');
    hold on;
    plot(dE_true, dN_true, 'b--', 'LineWidth', 2.0, 'DisplayName', 'Ground Truth');
    plot(dE_our, dN_our, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Estimated Trajectory');

    xlabel('East (m)');
    ylabel('North (m)');
    title('2D Trajectory Comparison (North vs East)');
    legend('show', 'Location', 'best');
    grid on;
    axis equal;
    hold off;

    % 2. Plot ENU Position Errors vs Time
    figure('Name', 'ENU Position Errors', 'NumberTitle', 'off');
    
    subplot(3,1,1);
    hold on;
    plot(second, ddN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'North Error (dN)');
    ylabel('dN (m)');
    title('Position Errors in ENU Frame');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;
    hold off;

    subplot(3,1,2);
    hold on;
    plot(second, ddE, 'g-', 'LineWidth', 1.5, 'DisplayName', 'East Error (dE)');
    ylabel('dE (m)');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;
    hold off;

    subplot(3,1,3);
    hold on;
    plot(second, ddU, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Up Error (dU)');
    xlabel('Time (s)');
    ylabel('dU (m)');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;
    hold off;
end

% RMSE calculation helper
function rmse_value = calculate_rmse(errors)
    rmse_value = sqrt(mean(errors.^2));
end