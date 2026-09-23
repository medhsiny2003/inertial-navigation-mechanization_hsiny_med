% ============================================================
% INS_Mechanization - Analysis Scripts
% Author: Mohammed Hsiny
% Field: Electrical Engineering & Industrial Control Student
% Institution: Faculty of Sciences and Techniques of Mohammedia
% Year: 2026
% Repository: inertial-navigation-mechanization_hsiny_med
% ============================================================

function RmseWithTime(data1, data2, window_size) 
    % data1 contains velocity and attitude errors
    % data2 contains ENU position trajectory information

    if nargin < 3
        window_size = 100;
    end

    % Extract arrays from data1
    second1 = table2array(data1(:,1));
    Vn = table2array(data1(:,2));
    Ve = table2array(data1(:,3));
    Vd = table2array(data1(:,4));

    roll = table2array(data1(:,8));
    pitch = table2array(data1(:,9));
    yaw = table2array(data1(:,10));

    % Extract arrays from data2
    second2 = table2array(data2(:,1));
    dN_true = table2array(data2(:,2));
    dE_true = table2array(data2(:,3));
    dU_true = table2array(data2(:,4));

    dN_our = table2array(data2(:,5));
    dE_our = table2array(data2(:,6));
    dU_our = table2array(data2(:,7));

    % Compute moving-window RMSE
    RMSE_vn = calculate_rmse_window(second1, Vn, window_size);
    RMSE_ve = calculate_rmse_window(second1, Ve, window_size);
    RMSE_vd = calculate_rmse_window(second1, Vd, window_size);

    RMSE_n = calculate_rmse_window(second2, dN_our - dN_true, window_size);
    RMSE_e = calculate_rmse_window(second2, dE_our - dE_true, window_size);
    RMSE_u = calculate_rmse_window(second2, dU_our - dU_true, window_size);

    RMSE_roll = calculate_rmse_window(second1, roll, window_size);
    RMSE_pitch = calculate_rmse_window(second1, pitch, window_size);
    RMSE_yaw = calculate_rmse_window(second1, yaw, window_size);
    
    % Terminal errors summary (Navigation performance metrics)
    mile = 1852.0; % 1 nautical mile = 1852 meters
    dN_last = dN_our(end) - dN_true(end);
    dE_last = dE_our(end) - dE_true(end);
    dU_last = dU_our(end) - dU_true(end);

    dVn_last = Vn(end);
    dVe_last = Ve(end);
    dVd_last = Vd(end);

    droll_last = roll(end);
    dpitch_last = pitch(end);
    dyaw_last = yaw(end);

    total_time1 = max((second1(end) - second1(1)) / 3600.0, 1e-6);
    total_time2 = max((second2(end) - second2(1)) / 3600.0, 1e-6);

    fprintf("Total duration: %0.4f hours (%0.2f seconds)\n\n", total_time1, second1(end) - second1(1));
    
    position_3d_error = sqrt(dN_last^2 + dE_last^2 + dU_last^2);
    fprintf("Position Drift Rate (m/h): dN=%0.4f, dE=%0.4f, dU=%0.4f, 3D=%0.4f\n", ...
        abs(dN_last/total_time2), abs(dE_last/total_time2), abs(dU_last/total_time2), ...
        position_3d_error/total_time2);
    fprintf("Position Drift Rate (nm/h): dN=%0.4f, dE=%0.4f, dU=%0.4f, 3D=%0.4f\n\n", ...
        abs(dN_last/total_time2)/mile, abs(dE_last/total_time2)/mile, abs(dU_last/total_time2)/mile, ...
        position_3d_error/total_time2/mile);

    velocity_3d_error = sqrt(dVn_last^2 + dVe_last^2 + dVd_last^2);
    fprintf("Velocity Drift (m/s / h): dVn=%0.4f, dVe=%0.4f, dVd=%0.4f, 3D=%0.4f\n", ...
        abs(dVn_last/total_time1), abs(dVe_last/total_time1), abs(dVd_last/total_time1), ...
        velocity_3d_error/total_time1);

    posture_3d_error = sqrt(droll_last^2 + dpitch_last^2 + dyaw_last^2);
    fprintf("Attitude Drift (deg/h): dRoll=%0.4f, dPitch=%0.4f, dYaw=%0.4f, 3D=%0.4f\n\n", ...
        abs(droll_last/total_time1), abs(dpitch_last/total_time1), abs(dyaw_last/total_time1), ...
        posture_3d_error/total_time1);

    % Plot RMSE figures
    figure('Name', 'RMSE Evolution Over Time', 'NumberTitle', 'off');

    % Velocity RMSE
    time_idx1 = 1:window_size:length(second1);
    subplot(3,3,1);  
    plot(second1(time_idx1), RMSE_vn, 'r-', 'LineWidth', 1.5);
    ylabel('Vn RMSE (m/s)');
    xlabel('Time (s)');
    title('North Velocity RMSE');
    grid on;
    xlim([second1(1), second1(end)]);

    subplot(3,3,2);  
    plot(second1(time_idx1), RMSE_ve, 'r-', 'LineWidth', 1.5);
    ylabel('Ve RMSE (m/s)');
    xlabel('Time (s)');
    title('East Velocity RMSE');
    grid on;
    xlim([second1(1), second1(end)]);

    subplot(3,3,3);  
    plot(second1(time_idx1), RMSE_vd, 'r-', 'LineWidth', 1.5);
    ylabel('Vd RMSE (m/s)');
    xlabel('Time (s)');
    title('Down Velocity RMSE');
    grid on;
    xlim([second1(1), second1(end)]);

    % Position RMSE
    time_idx2 = 1:window_size:length(second2);
    subplot(3,3,4);  
    plot(second2(time_idx2), RMSE_n, 'g-', 'LineWidth', 1.5);
    ylabel('dN RMSE (m)');
    xlabel('Time (s)');
    title('North Error RMSE');
    grid on;
    xlim([second2(1), second2(end)]);

    subplot(3,3,5);  
    plot(second2(time_idx2), RMSE_e, 'g-', 'LineWidth', 1.5);
    ylabel('dE RMSE (m)');
    xlabel('Time (s)');
    title('East Error RMSE');
    grid on;
    xlim([second2(1), second2(end)]);

    subplot(3,3,6);  
    plot(second2(time_idx2), RMSE_u, 'g-', 'LineWidth', 1.5);
    ylabel('dU RMSE (m)');
    xlabel('Time (s)');
    title('Up Error RMSE');
    grid on;
    xlim([second2(1), second2(end)]);

    % Attitude RMSE
    subplot(3,3,7);  
    plot(second1(time_idx1), RMSE_roll, 'b-', 'LineWidth', 1.5);
    ylabel('Roll RMSE (deg)');
    xlabel('Time (s)');
    title('Roll RMSE');
    grid on;
    xlim([second1(1), second1(end)]);

    subplot(3,3,8);  
    plot(second1(time_idx1), RMSE_pitch, 'b-', 'LineWidth', 1.5);
    ylabel('Pitch RMSE (deg)');
    xlabel('Time (s)');
    title('Pitch RMSE');
    grid on;
    xlim([second1(1), second1(end)]);

    subplot(3,3,9);  
    plot(second1(time_idx1), RMSE_yaw, 'b-', 'LineWidth', 1.5);
    ylabel('Yaw RMSE (deg)');
    xlabel('Time (s)');
    title('Yaw RMSE');
    grid on;
    xlim([second1(1), second1(end)]);
end

function RMSE_values = calculate_rmse_window(time, errors, window_size)
    N = length(errors);
    num_windows = ceil(N / window_size);
    RMSE_values = zeros(num_windows, 1);
    
    for i = 1:num_windows
        start_idx = (i - 1) * window_size + 1;
        end_idx = min(i * window_size, N);
        window_errors = errors(start_idx:end_idx);
        RMSE_values(i) = sqrt(mean(window_errors.^2));
    end
end
