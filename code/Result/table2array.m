% ============================================================
% INS_Mechanization - Analysis Scripts
% Author: Mohammed Hsiny
% Field: Electrical Engineering & Industrial Control Student
% Institution: Faculty of Sciences and Techniques of Mohammedia
% Year: 2026
% Repository: inertial-navigation-mechanization_hsiny_med
% ============================================================

function A = table2array(X)
    if ismatrix(X) && isnumeric(X)
        A = X;
    elseif iscell(X)
        A = cell2mat(X);
    else
        A = double(X);
    end
end
