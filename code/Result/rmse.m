% ============================================================
% INS_Mechanization - Analysis Script
% Author: Mohammed Hsiny
% Field: Electrical Engineering & Industrial Control Student
% Institution: Faculty of Sciences and Techniques of Mohammedia
% Year: 2026
% Repository: inertial-navigation-mechanization_hsiny_med
% ============================================================

function r = rmse(estimated, truth)
    if nargin == 1
        % Single argument: root mean square of error array
        r = sqrt(mean(estimated.^2));
    else
        % Two arguments: root mean square error between estimated and truth
        r = sqrt(mean((estimated - truth).^2));
    end
end
