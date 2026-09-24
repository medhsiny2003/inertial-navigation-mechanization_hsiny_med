% ============================================================
% INS_Mechanization - Analysis Script
% Author: Mohammed Hsiny
% Field: Electrical Engineering & Industrial Control Student
% Institution: Faculty of Sciences and Techniques of Mohammedia
% Year: 2026
% Repository: inertial-navigation-mechanization_hsiny_med
% ============================================================

function DataFig(data_our, data_true)
    d_our = data_our;
    d_true = data_true;

    second_our = d_our(:, 1);
    B_our = d_our(:, 2);
    L_our = d_our(:, 3);
    H_our = d_our(:, 4);
    Vn_our = d_our(:, 5);
    Ve_our = d_our(:, 6);
    Vd_our = d_our(:, 7);
    roll_our = d_our(:, 8);
    pitch_our = d_our(:, 9);
    yaw_our = d_our(:, 10);

    B_true = d_true(:, 2);
    L_true = d_true(:, 3);
    H_true = d_true(:, 4);
    Vn_true = d_true(:, 5);
    Ve_true = d_true(:, 6);
    Vd_true = d_true(:, 7);
    roll_true = d_true(:, 8);
    pitch_true = d_true(:, 9);
    yaw_true = d_true(:, 10);

    if ~exist('Figures', 'dir')
        mkdir('Figures');
    end

    % ------------------------------------------------------------
    % Figure 1: Altitude Profile
    % ------------------------------------------------------------
    h_alt = figure('Name', 'Altitude Profile', 'NumberTitle', 'off');
    set(h_alt, 'Color', 'w');
    plot(second_our, H_our, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(second_our, H_true, 'r--', 'LineWidth', 1.5);
    grid on;
    box on;
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Altitude (m)', 'FontSize', 11);
    title('Altitude Profile: Estimated vs Ground Truth', 'FontSize', 12, 'FontWeight', 'bold');
    h_leg = legend('Estimated', 'Ground Truth', 'Location', 'best');
    set(h_leg, 'FontSize', 10);
    set(gca, 'FontSize', 11);
    text(0.98, 0.02, 'Mohammed Hsiny - FST Mohammedia - 2026', ...
         'Units', 'normalized', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 9, ...
         'Color', [0.4 0.4 0.4]);
    hold off;
    print(h_alt, '-dpng', '-r300', 'Figures/altitude_profile.png');

    % ------------------------------------------------------------
    % Figure 2: Attitude Angles (Roll, Pitch, Yaw)
    % ------------------------------------------------------------
    h_att = figure('Name', 'Attitude Angles: Roll, Pitch, Yaw', 'NumberTitle', 'off');
    set(h_att, 'Color', 'w');

    subplot(3, 1, 1);
    plot(second_our, roll_our, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(second_our, roll_true, 'r--', 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Roll (deg)', 'FontSize', 11);
    title('Roll Angle: Estimated vs Ground Truth', 'FontSize', 11, 'FontWeight', 'bold');
    h_l1 = legend('Estimated', 'Ground Truth', 'Location', 'best');
    set(h_l1, 'FontSize', 9);
    set(gca, 'FontSize', 10);
    xlim([min(second_our), max(second_our)]);
    hold off;

    subplot(3, 1, 2);
    plot(second_our, pitch_our, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(second_our, pitch_true, 'r--', 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Pitch (deg)', 'FontSize', 11);
    title('Pitch Angle: Estimated vs Ground Truth', 'FontSize', 11, 'FontWeight', 'bold');
    h_l2 = legend('Estimated', 'Ground Truth', 'Location', 'best');
    set(h_l2, 'FontSize', 9);
    set(gca, 'FontSize', 10);
    xlim([min(second_our), max(second_our)]);
    hold off;

    subplot(3, 1, 3);
    plot(second_our, yaw_our, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(second_our, yaw_true, 'r--', 'LineWidth', 1.5);
    grid on; box on;
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Yaw (deg)', 'FontSize', 11);
    title('Yaw / Heading Angle: Estimated vs Ground Truth', 'FontSize', 11, 'FontWeight', 'bold');
    h_l3 = legend('Estimated', 'Ground Truth', 'Location', 'best');
    set(h_l3, 'FontSize', 9);
    set(gca, 'FontSize', 10);
    xlim([min(second_our), max(second_our)]);
    text(0.98, 0.02, 'Mohammed Hsiny - FST Mohammedia - 2026', ...
         'Units', 'normalized', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 9, ...
         'Color', [0.4 0.4 0.4]);
    hold off;
    print(h_att, '-dpng', '-r300', 'Figures/attitude_angles.png');

    % ------------------------------------------------------------
    % Figure 3: Full 9-Subplot Overview
    % ------------------------------------------------------------
    h_main = figure('Name', 'INS Mechanization - Trajectory and Attitude Estimation', 'NumberTitle', 'off');
    set(h_main, 'Color', 'w');

    subplot(3, 3, 1);
    plot(second_our, Vn_our, 'b-', 'LineWidth', 1.5);
    hold on; plot(second_our, Vn_true, 'r--', 'LineWidth', 1.5);
    grid on; box on; ylabel('Vn (m/s)', 'FontSize', 10);
    title('North Velocity', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Estimated', 'Ground Truth', 'Location', 'best');
    xlim([min(second_our), max(second_our)]); set(gca, 'FontSize', 9); hold off;

    subplot(3, 3, 2);
    plot(second_our, Ve_our, 'b-', 'LineWidth', 1.5);
    hold on; plot(second_our, Ve_true, 'r--', 'LineWidth', 1.5);
    grid on; box on; ylabel('Ve (m/s)', 'FontSize', 10);
    title('East Velocity', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Estimated', 'Ground Truth', 'Location', 'best');
    xlim([min(second_our), max(second_our)]); set(gca, 'FontSize', 9); hold off;

    subplot(3, 3, 3);
    plot(second_our, Vd_our, 'b-', 'LineWidth', 1.5);
    hold on; plot(second_our, Vd_true, 'r--', 'LineWidth', 1.5);
    grid on; box on; ylabel('Vd (m/s)', 'FontSize', 10);
    title('Down Velocity', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Estimated', 'Ground Truth', 'Location', 'best');
    xlim([min(second_our), max(second_our)]); set(gca, 'FontSize', 9); hold off;

    subplot(3, 3, 4);
    plot(second_our, B_our, 'b-', 'LineWidth', 1.5);
    hold on; plot(second_our, B_true, 'r--', 'LineWidth', 1.5);
    grid on; box on; ylabel('Latitude (deg)', 'FontSize', 10);
    title('Latitude Profile', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Estimated', 'Ground Truth', 'Location', 'best');
    xlim([min(second_our), max(second_our)]); set(gca, 'FontSize', 9); hold off;

    subplot(3, 3, 5);
    plot(second_our, L_our, 'b-', 'LineWidth', 1.5);
    hold on; plot(second_our, L_true, 'r--', 'LineWidth', 1.5);
    grid on; box on; ylabel('Longitude (deg)', 'FontSize', 10);
    title('Longitude Profile', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Estimated', 'Ground Truth', 'Location', 'best');
    xlim([min(second_our), max(second_our)]); set(gca, 'FontSize', 9); hold off;

    subplot(3, 3, 6);
    plot(second_our, H_our, 'b-', 'LineWidth', 1.5);
    hold on; plot(second_our, H_true, 'r--', 'LineWidth', 1.5);
    grid on; box on; ylabel('Altitude (m)', 'FontSize', 10);
    title('Altitude Profile', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Estimated', 'Ground Truth', 'Location', 'best');
    xlim([min(second_our), max(second_our)]); set(gca, 'FontSize', 9); hold off;

    subplot(3, 3, 7);
    plot(second_our, roll_our, 'b-', 'LineWidth', 1.5);
    hold on; plot(second_our, roll_true, 'r--', 'LineWidth', 1.5);
    grid on; box on; xlabel('Time (s)', 'FontSize', 10); ylabel('Roll (deg)', 'FontSize', 10);
    title('Roll Angle', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Estimated', 'Ground Truth', 'Location', 'best');
    xlim([min(second_our), max(second_our)]); set(gca, 'FontSize', 9); hold off;

    subplot(3, 3, 8);
    plot(second_our, pitch_our, 'b-', 'LineWidth', 1.5);
    hold on; plot(second_our, pitch_true, 'r--', 'LineWidth', 1.5);
    grid on; box on; xlabel('Time (s)', 'FontSize', 10); ylabel('Pitch (deg)', 'FontSize', 10);
    title('Pitch Angle', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Estimated', 'Ground Truth', 'Location', 'best');
    xlim([min(second_our), max(second_our)]); set(gca, 'FontSize', 9); hold off;

    subplot(3, 3, 9);
    plot(second_our, yaw_our, 'b-', 'LineWidth', 1.5);
    hold on; plot(second_our, yaw_true, 'r--', 'LineWidth', 1.5);
    grid on; box on; xlabel('Time (s)', 'FontSize', 10); ylabel('Yaw (deg)', 'FontSize', 10);
    title('Yaw / Heading Angle', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Estimated', 'Ground Truth', 'Location', 'best');
    xlim([min(second_our), max(second_our)]); set(gca, 'FontSize', 9);
    text(0.98, 0.02, 'Mohammed Hsiny - FST Mohammedia - 2026', ...
         'Units', 'normalized', ...
         'HorizontalAlignment', 'right', ...
         'FontSize', 8, ...
         'Color', [0.4 0.4 0.4]);
    hold off;
end
