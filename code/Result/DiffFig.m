% ============================================================
% INS_Mechanization - Analysis Scripts
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

    % Convert table columns to numeric arrays
    second = table2array(data(:,1));
    Vn = table2array(data(:,2));
    Ve = table2array(data(:,3));
    Vd = table2array(data(:,4));

    B = table2array(data(:,5));
    L = table2array(data(:,6));
    H = table2array(data(:,7));

    roll = table2array(data(:,8));
    pitch = table2array(data(:,9));
    yaw = table2array(data(:,10));

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

    % Plot Errors: Velocity (Row 1), Position (Row 2), Attitude (Row 3)
    figure('Name', 'Estimation Error (Differences)', 'NumberTitle', 'off');

    % Velocity Errors
    subplot(3,3,1);    
    plot(second, Vn, 'r-', 'LineWidth', 1.5, 'DisplayName', 'dVn');
    ylabel('dVn (m/s)');
    title('North Velocity Error');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;

    subplot(3,3,2);
    plot(second, Ve, 'r-', 'LineWidth', 1.5, 'DisplayName', 'dVe');
    ylabel('dVe (m/s)');
    title('East Velocity Error');
    legend('show', 'Location', 'best');    
    xlim([min(second), max(second)]);
    grid on;

    subplot(3,3,3);
    plot(second, Vd, 'r-', 'LineWidth', 1.5, 'DisplayName', 'dVd');
    ylabel('dVd (m/s)');
    title('Down Velocity Error');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;

    % Position Errors
    subplot(3,3,4);
    plot(second, B, 'g-', 'LineWidth', 1.5, 'DisplayName', 'dLat');
    ylabel('dLat (deg)');
    title('Latitude Error');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;

    subplot(3,3,5);
    plot(second, L, 'g-', 'LineWidth', 1.5, 'DisplayName', 'dLon');
    ylabel('dLon (deg)');
    title('Longitude Error');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;

    subplot(3,3,6);
    plot(second, H, 'g-', 'LineWidth', 1.5, 'DisplayName', 'dH');
    ylabel('dH (m)');
    title('Height Error');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;

    % Attitude Errors
    subplot(3,3,7);
    plot(second, roll, 'b-', 'LineWidth', 1.5, 'DisplayName', 'dRoll');
    xlabel('Time (s)');
    ylabel('dRoll (deg)');
    title('Roll Error');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;

    subplot(3,3,8);
    plot(second, pitch, 'b-', 'LineWidth', 1.5, 'DisplayName', 'dPitch');
    xlabel('Time (s)');
    ylabel('dPitch (deg)');
    title('Pitch Error');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;

    subplot(3,3,9);
    plot(second, yaw, 'b-', 'LineWidth', 1.5, 'DisplayName', 'dYaw');
    xlabel('Time (s)');
    ylabel('dYaw (deg)');
    title('Yaw Error');
    legend('show', 'Location', 'best');
    xlim([min(second), max(second)]);
    grid on;
end

function rmse_value = calculate_rmse(errors)
    rmse_value = sqrt(mean(errors.^2));
end
