% ============================================================
% INS_Mechanization - Analysis Script
% Author: Mohammed Hsiny
% Field: Electrical Engineering & Industrial Control Student
% Institution: Faculty of Sciences and Techniques of Mohammedia
% Year: 2026
% Repository: inertial-navigation-mechanization_hsiny_med
% ============================================================

function data = ReadFile(filename, mode)
    if ~exist(filename, 'file')
        error('Cannot find file: %s', filename);
    end

    switch (mode)
        case 0 % Read difference or mechanization result data (1 header line)
            data = dlmread(filename, ' ', 1, 0);

        case 1 % Read reference truth data (0 header lines)
            try
                data = dlmread(filename, ' ', 0, 0);
            catch
                data = dlmread(filename, ',', 0, 0);
            end

        case 2 % Read dENU trajectory data (1 header line)
            data = dlmread(filename, ' ', 1, 0);
    end
end
