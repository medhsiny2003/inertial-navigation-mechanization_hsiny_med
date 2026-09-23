% ============================================================
% INS_Mechanization - Analysis Scripts
% Author: Mohammed Hsiny
% Field: Electrical Engineering & Industrial Control Student
% Institution: Faculty of Sciences and Techniques of Mohammedia
% Year: 2026
% Repository: inertial-navigation-mechanization_hsiny_med
% ============================================================

function DataFig(data_our, data_true)
    % Convert table columns to numeric arrays
    second_our = table2array(data_our(:,1));
    B_our = table2array(data_our(:,2));
    L_our = table2array(data_our(:,3));
    H_our = table2array(data_our(:,4));

    Vn_our = table2array(data_our(:,5));
    Ve_our = table2array(data_our(:,6));
    Vd_our = table2array(data_our(:,7));

    roll_our = table2array(data_our(:,8));
    pitch_our = table2array(data_our(:,9));
    yaw_our = table2array(data_our(:,10));

    % Reference ground truth columns
    B_true = table2array(data_true(:,2));
    L_true = table2array(data_true(:,3));
    H_true = table2array(data_true(:,4));

    Vn_true = table2array(data_true(:,5));
    Ve_true = table2array(data_true(:,6));
    Vd_true = table2array(data_true(:,7));

    roll_true = table2array(data_true(:,8));
    pitch_true = table2array(data_true(:,9));
    yaw_true = table2array(data_true(:,10));

    %% Subplot Grid Layout: Velocity (Row 1), Position (Row 2), Attitude (Row 3)
    figure('Name', 'Estimated vs Reference Comparison', 'NumberTitle', 'off');

    % Row 1: Velocities (Vn, Ve, Vd)
    subplot(3,3,1);
    hold on;
    plot(second_our, Vn_our, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vn (Estimated)');
    plot(second_our, Vn_true, 'b--', 'LineWidth', 1.0, 'DisplayName', 'Vn (True)');
    ylabel('Vn (m/s)');
    title('North Velocity');
    legend('show', 'Location', 'best');
    xlim([min(second_our), max(second_our)]);
    grid on;
    hold off;

    subplot(3,3,2);
    hold on;
    plot(second_our, Ve_our, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ve (Estimated)');
    plot(second_our, Ve_true, 'b--', 'LineWidth', 1.0, 'DisplayName', 'Ve (True)');
    ylabel('Ve (m/s)');
    title('East Velocity');
    legend('show', 'Location', 'best');
    xlim([min(second_our), max(second_our)]);
    grid on;
    hold off;

    subplot(3,3,3);
    hold on;
    plot(second_our, Vd_our, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vd (Estimated)');
    plot(second_our, Vd_true, 'b--', 'LineWidth', 1.0, 'DisplayName', 'Vd (True)');
    ylabel('Vd (m/s)');
    title('Down Velocity');
    legend('show', 'Location', 'best');
    xlim([min(second_our), max(second_our)]);
    grid on;
    hold off;

    % Row 2: Position (Latitude, Longitude, Height)
    subplot(3,3,4);
    hold on;
    plot(second_our, B_our, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Lat (Estimated)');
    plot(second_our, B_true, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Lat (True)');
    ylabel('Latitude (deg)');
    title('Latitude');
    legend('show', 'Location', 'best');
    xlim([min(second_our), max(second_our)]);
    grid on;
    hold off;

    subplot(3,3,5);
    hold on;
    plot(second_our, L_our, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Lon (Estimated)');
    plot(second_our, L_true, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Lon (True)');
    ylabel('Longitude (deg)');
    title('Longitude');
    legend('show', 'Location', 'best');
    xlim([min(second_our), max(second_our)]);
    grid on;
    hold off;

    subplot(3,3,6);
    hold on;
    plot(second_our, H_our, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Height (Estimated)');
    plot(second_our, H_true, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Height (True)');
    ylabel('Height (m)');
    title('Height / Altitude');
    legend('show', 'Location', 'best');
    xlim([min(second_our), max(second_our)]);
    grid on;
    hold off;

    % Row 3: Attitude (Roll, Pitch, Yaw)
    subplot(3,3,7);
    hold on;
    plot(second_our, roll_our, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Roll (Estimated)');
    plot(second_our, roll_true, 'c--', 'LineWidth', 1.0, 'DisplayName', 'Roll (True)');
    ylabel('Roll (deg)');
    xlabel('Time (s)');
    title('Roll Angle');
    legend('show', 'Location', 'best');
    xlim([min(second_our), max(second_our)]);
    grid on;
    hold off;

    subplot(3,3,8);
    hold on;
    plot(second_our, pitch_our, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Pitch (Estimated)');
    plot(second_our, pitch_true, 'c--', 'LineWidth', 1.0, 'DisplayName', 'Pitch (True)');
    ylabel('Pitch (deg)');
    xlabel('Time (s)');
    title('Pitch Angle');
    legend('show', 'Location', 'best');
    xlim([min(second_our), max(second_our)]);
    grid on;
    hold off;

    subplot(3,3,9);
    hold on;
    plot(second_our, yaw_our, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Yaw (Estimated)');
    plot(second_our, yaw_true, 'c--', 'LineWidth', 1.0, 'DisplayName', 'Yaw (True)');
    ylabel('Yaw (deg)');
    xlabel('Time (s)');
    title('Yaw / Heading Angle');
    legend('show', 'Location', 'best');
    xlim([min(second_our), max(second_our)]);
    grid on;
    hold off;
end
