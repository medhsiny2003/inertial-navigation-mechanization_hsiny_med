% ============================================================
% INS_Mechanization - Analysis Scripts
% Author: Mohammed Hsiny
% Field: Electrical Engineering & Industrial Control Student
% Institution: Faculty of Sciences and Techniques of Mohammedia
% Year: 2026
% Repository: inertial-navigation-mechanization_hsiny_med
% ============================================================

% Prompt user for selection
choice = input(['Please choose analysis option:\n' ...
    '     0 Compute initial average velocity, position, and attitude\n' ...
    '     1 Plot difference between estimated results and ground truth\n' ...
    '     2 Plot comparison between estimated results and ground truth\n' ...
    '     3 Plot 2D/3D trajectory comparison\n' ...
    '     4 Plot RMSE evolution over time\n' ...
    '     5 Plot example reference data difference and comparison\n' ...
    'Enter choice (0-5): ']);

if (~(choice == 0 || choice == 1 || choice == 2 || choice == 3 || choice == 4 || choice == 5))
    fprintf("\nUnrecognized choice: %d. Exiting program!\n", choice);
end

switch (choice)
    case 0
        % Read data
        truthdata_file_path = "truth.nav";
        if ~exist(truthdata_file_path, 'file')
            truthdata_file_path = "../Data/TruthOne.nav";
        end
        truthdata = ReadFile(truthdata_file_path, 1);
        
        % Extract data columns
        second = table2array(truthdata(:, 2));
        B = table2array(truthdata(:, 3));
        L = table2array(truthdata(:, 4));
        H = table2array(truthdata(:, 5));
        
        roll = table2array(truthdata(:, 9));
        pitch = table2array(truthdata(:, 10));
        yaw = table2array(truthdata(:, 11));
        
        time_threshold = 440532.000;
        index_before_threshold = (second < time_threshold);
        if sum(index_before_threshold) == 0
            index_before_threshold = true(size(second));
        end
        
        B_avg = mean(B(index_before_threshold));
        L_avg = mean(L(index_before_threshold));
        H_avg = mean(H(index_before_threshold));
        roll_avg = mean(roll(index_before_threshold));
        pitch_avg = mean(pitch(index_before_threshold));
        yaw_avg = mean(yaw(index_before_threshold));
        
        fprintf('Average Latitude (B): %.10f\n', B_avg);
        fprintf('Average Longitude (L): %.10f\n', L_avg);
        fprintf('Average Height (H): %.10f\n', H_avg);
        fprintf('Average Roll: %.10f\n', roll_avg);
        fprintf('Average Pitch: %.10f\n', pitch_avg);
        fprintf('Average Yaw: %.10f\n', yaw_avg);

    case 1    
        dresult_file_path = "diff_result.txt";
        dresult_data = ReadFile(dresult_file_path, 0);
        DiffFig(dresult_data, 1);
    
    case 2
        ours_result_file_path = "result.txt";
        truthdata_file_path = "true.txt";

        ours_data = ReadFile(ours_result_file_path, 0);
        true_data = ReadFile(truthdata_file_path, 0);

        DataFig(ours_data, true_data);

    case 3
        denu_file_path = "denu_result.txt";
        denu_data = ReadFile(denu_file_path, 2);
        dEnuFig(denu_data);
   
    case 4
        dresult_file_path = "diff_result.txt";
        denu_file_path = "denu_result.txt";

        dresult_data = ReadFile(dresult_file_path, 0);
        denu_data = ReadFile(denu_file_path, 2);

        RmseWithTime(dresult_data, denu_data, 100);

    case 5
        ins_result_file_path = "ExampleResult/INS_Result.txt";
        true_file_path = "ExampleResult/INS_True.txt";
        ins_diff_result_file_path = "ExampleResult/INS_Diff_Result.txt";
        denu_file_path = "ExampleResult/denu_Result.txt";

        ins_data = ReadFile(ins_result_file_path, 0);
        true_data = ReadFile(true_file_path, 0);
        ins_diff_data = ReadFile(ins_diff_result_file_path, 0);
        denu_data = ReadFile(denu_file_path, 2);

        DataFig(ins_data, true_data);
        DiffFig(ins_diff_data, 0);
        dEnuFig(denu_data);
end